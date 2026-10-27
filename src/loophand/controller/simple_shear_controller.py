import time
import logging
import numpy as np
import pinocchio as pin
from collections import deque
from typing import Dict, List, Optional
import multiprocessing as mp

# 引入硬件驱动和控制算法
from aero_open_sdk.aero_hand import AeroHand
from loophand.driver.tactile_driver import TactileSensor
from loophand.controller.pid import BoundedPID
from loophand.controller.admittance import AdmittanceController
from loophand.viz.shear_debugger import ShearDebugger
from loophand.paths import URDF_PATH

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("ShearControl")

# ==============================================================================
# 1. 硬件配置 & 常量 (保持不变)
# ==============================================================================
SENSOR_TO_LINK_MAP = {
    "Thumb-Tip": "left_thumb_tip_link",
    "Index-Tip": "left_index_tip_link",
    "Index-Mid": "left_index_proximal_link", 
    "Middle-Tip": "left_middle_tip_link",
    "Middle-Mid": "left_middle_proximal_link", 
    "Ring-Tip": "left_ring_tip_link",
    "Ring-Mid": "left_ring_proximal_link", 
    "Pinky-Tip": "left_pinky_tip_link",
    "Pinky-Mid": "left_pinky_proximal_link" 
}

# 偏移量：用于将箭头绘制在指腹表面，而不是关节中心
SENSOR_OFFSETS = {
    "Tip": np.array([0.0, 0.0, 0.0]), 
    "Mid": np.array([0.0, 0.0, 0.025]) 
}

FINGER_GROUPS = {
    "Thumb":  ["Thumb-Tip"],
    "Index":  ["Index-Tip", "Index-Mid"],
    "Middle": ["Middle-Tip", "Middle-Mid"],
    "Ring":   ["Ring-Tip",  "Ring-Mid"],
    "Pinky":  ["Pinky-Tip", "Pinky-Mid"]
}

JOINT_ORDER_FULL = [
    "left_thumb_cmc_abd", "left_thumb_cmc_flex", "left_thumb_mcp", "left_thumb_ip",
    "left_index_mcp_flex", "left_index_pip", "left_index_dip",
    "left_middle_mcp_flex", "left_middle_pip", "left_middle_dip",
    "left_ring_mcp_flex", "left_ring_pip", "left_ring_dip",
    "left_pinky_mcp_flex", "left_pinky_pip", "left_pinky_dip"
]

ACTIVE_JOINT_NAMES = [
    "left_thumb_cmc_abd", "left_thumb_cmc_flex", "left_thumb_mcp",
    "left_index_mcp_flex", "left_middle_mcp_flex", "left_ring_mcp_flex", "left_pinky_mcp_flex"
]

# ==============================================================================
# 2. 信号处理 (保持不变)
# ==============================================================================
class SignalProcessor:
    def __init__(self, alpha=0.4, history_len=5, derivative_alpha=0.1):
        self.alpha = alpha
        self.derivative_alpha = derivative_alpha
        self.history_len = history_len
        self.epsilon = 1e-3
        self.lowpass = np.zeros(3)
        self.history = deque(maxlen=history_len)
        self.shear_prev = np.zeros(3)
        
    def process(self, raw_force: np.ndarray, dt: float):
        self.lowpass = self.alpha * raw_force + (1.0 - self.alpha) * self.lowpass
        self.history.append(self.lowpass.copy())
        
        if len(self.history) >= 2:
            prev = self.history[0]
            shear_delta = (self.lowpass - prev) / max(dt * (len(self.history) - 1), self.epsilon)
        else:
            shear_delta = np.zeros(3)
            
        self.shear_prev = (self.derivative_alpha * shear_delta + (1.0 - self.derivative_alpha) * self.shear_prev)
        
        tangential_force = np.linalg.norm(self.lowpass[:2])
        normal_force = self.lowpass[2]
        slip_ratio = tangential_force / max(abs(normal_force), self.epsilon)
        
        return {
            "force_filtered": self.lowpass.copy(),
            "shear_rate": self.shear_prev.copy(),
            "normal_force": normal_force,
            "tangential_force": tangential_force,
            "slip_ratio": slip_ratio
        }

# ==============================================================================
# 3. 控制器主类 (核心修改)
# ==============================================================================
class ShearGraspController:
    # 修改 1: 构造函数接收 gui_queue 和 cmd_queue
    def __init__(self, urdf_path: str, port_hand="/dev/ttyACM1", port_tactile="/dev/ttyACM0", gui_queue=None, cmd_queue=None):
        logger.info(f"Init Hardware (Hand:{port_hand}, Tactile:{port_tactile})...")
        
        # 硬件初始化
        self.hand = AeroHand(port=port_hand) 
        self.tactile = TactileSensor(port=port_tactile)

        logger.info("正在标定触觉传感器(4s)...")
        time.sleep(1.0) 
        self.tactile.tare(duration=4.0)
        
        # [修改] 不再启动 VisualizationServer (WebSocket)，改用 Queue
        self.gui_queue = gui_queue
        self.cmd_queue = cmd_queue

        # 启动调参器 (ShearDebugger)
        # 注意：ShearDebugger 是独立的进程，这里启动没问题
        self.debugger = ShearDebugger()

        self.current_pid_params = [8.0, 0.5, 0.0] 
        self.current_adm_params = [0.02, 20.0]    
        self.debug_target_sensor = "Index-Mid"
        
        # Pinocchio 模型
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        # 索引映射
        self.pin_active_indices = []
        self.user_active_indices = []
        for name in ACTIVE_JOINT_NAMES:
            if self.model.existJointName(name):
                joint_id = self.model.getJointId(name)
                self.pin_active_indices.append(self.model.idx_vs[joint_id])
            if name in JOINT_ORDER_FULL:
                self.user_active_indices.append(JOINT_ORDER_FULL.index(name))
        
        self.dt = 0.01
        self._gravity_world = np.array([0.0, 0.0, -1.0], dtype=float)
        
        # 初始化控制模块
        self.modules = {}
        PID_PARAMS = (8.0, 0.5, 0.0, 5.0) 
        ADM_M = np.array([0.02, 0.02, 0.02]) 
        ADM_D = np.array([1.5, 1.5, 1.5]) 
        ADM_K = np.array([0.0, 0.0, 0.0])

        for sensor_name, link_name in SENSOR_TO_LINK_MAP.items():
            self.modules[sensor_name] = {
                "link_name": link_name,
                "processor": SignalProcessor(),
                "pid": BoundedPID(*PID_PARAMS),
                "admittance": AdmittanceController(ADM_M, ADM_K, ADM_D, self.dt),
                "contact_active": False
            }
        
        self.current_q = np.zeros(len(JOINT_ORDER_FULL))
        self.running = False

    def _map_7_to_16(self, compact_deg: List[float]) -> np.ndarray:
        # ... (映射代码保持不变) ...
        q_7 = np.deg2rad(np.array(compact_deg))
        q_16 = np.zeros(len(JOINT_ORDER_FULL))
        q_16[0] = q_7[0]; q_16[1] = q_7[1]; q_16[2] = q_7[2]; q_16[3] = q_7[2] * 1.0 
        val_idx = q_7[3]; q_16[4] = val_idx; q_16[5] = val_idx * 0.8; q_16[6] = val_idx * 0.6 
        val_mid = q_7[4]; q_16[7] = val_mid; q_16[8] = val_mid * 0.8; q_16[9] = val_mid * 0.6
        val_rng = q_7[5]; q_16[10] = val_rng; q_16[11] = val_rng * 0.8; q_16[12] = val_rng * 0.6
        val_pnk = q_7[6]; q_16[13] = val_pnk; q_16[14] = val_pnk * 0.8; q_16[15] = val_pnk * 0.6
        return np.array(q_16, dtype=np.float64)

    def _update_kinematics(self, q_user: np.ndarray, link_name: str):
        # ... (运动学代码保持不变) ...
        q_pin = pin.neutral(self.model)
        for user_i, name in enumerate(JOINT_ORDER_FULL):
            if self.model.existJointName(name):
                pin_id = self.model.getJointId(name)
                pin_q_idx = self.model.idx_qs[pin_id]
                q_pin[pin_q_idx] = q_user[user_i]
        
        pin.forwardKinematics(self.model, self.data, q_pin)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data, q_pin)
        
        frame_id = self.model.getFrameId(link_name)
        J_spatial = pin.getFrameJacobian(self.model, self.data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        
        J_lin_origin = np.array(J_spatial[:3, :], copy=False)
        J_ang = np.array(J_spatial[3:, :], copy=False)
        rot = np.array(self.data.oMf[frame_id].rotation, copy=False)
        
        if "proximal" in link_name:
             local_offset = np.array([0.0, 0.0, 0.025]) 
        else:
             local_offset = np.array([0.0, 0.0, 0.0])
        
        r_world = rot @ local_offset
        rx, ry, rz = r_world
        skew_r = np.array([[0, -rz, ry], [rz, 0, -rx], [-ry, rx, 0]])
        J_lin_sensor = J_lin_origin - skew_r @ J_ang
        return rot, J_lin_sensor

    def _gravity_scaling(self, rotation_ws: np.ndarray) -> np.ndarray:
        # ... (重力代码保持不变) ...
        def axis_gain(axis):
            axis_unit = axis / max(np.linalg.norm(axis), 1e-9)
            cos_theta = np.clip(abs(float(axis_unit.dot(self._gravity_world))), 0.0, 1.0)
            theta = float(np.arccos(cos_theta))
            return float(np.clip(1.0 - (2.0 / np.pi) * theta, 0.0, 1.0))
        return np.array([axis_gain(rotation_ws[:, 0]), axis_gain(rotation_ws[:, 1])])

    def _solve_ik_underactuated(self, J_full: np.ndarray, dx: np.ndarray):
        # ... (IK 代码保持不变) ...
        J_active = J_full[:, self.pin_active_indices]
        dls_lambda = 0.05
        try:
            inv_term = np.linalg.inv(J_active @ J_active.T + dls_lambda**2 * np.eye(3))
            dq_active = J_active.T @ inv_term @ dx
        except np.linalg.LinAlgError:
            return np.zeros(len(JOINT_ORDER_FULL))
        
        dq_full = np.zeros(len(JOINT_ORDER_FULL))
        for i, val in enumerate(dq_active):
            user_idx = self.user_active_indices[i]
            dq_full[user_idx] = val
            if user_idx in [4, 7, 10, 13]: 
                if user_idx + 1 < len(dq_full): dq_full[user_idx+1] = val * 0.8
                if user_idx + 2 < len(dq_full): dq_full[user_idx+2] = val * 0.6
        return dq_full

    def step(self):
        # 1. 调参模块接收
        new_params = self.debugger.get_updated_params()
        if new_params:
            target = new_params.get('target', "Index-Mid")
            self.debug_target_sensor = target
            
            if target in self.modules:
                module = self.modules[target]
                module['pid'].kp = new_params['kp']
                module['pid'].ki = new_params['ki']
                module['pid'].kd = new_params['kd']
                module['admittance']._mass = np.array([new_params['m']] * 3)
                module['admittance']._damping = np.array([new_params['d']] * 3)

        # 2. 硬件读取
        try:
            joint_deg_7 = self.hand.get_joint_positions_compact()
        except Exception: return
        if joint_deg_7 is None: return
        self.current_q = self._map_7_to_16(joint_deg_7)
        if np.isnan(self.current_q).any(): return

        tactile_data = self.tactile.get_latest_data()    
        total_dq = np.zeros(len(self.current_q))
        
        # [修改] 用于发送给 GUI 的数据容器
        gui_forces = {}
        gui_kinematics = {}

        # 3. 循环处理
        for finger_name, sensor_names in FINGER_GROUPS.items():
            finger_entries = []
            
            for s_name in sensor_names:
                if s_name not in tactile_data: continue
                module = self.modules[s_name]
                last_pid_out = module.get('last_pid_val', 0.0)
                
                rot, J = self._update_kinematics(self.current_q, module['link_name'])
                
                # [新增] 获取 Link 的位置 (Translation)，用于 GUI 绘制箭头起点
                frame_id = self.model.getFrameId(module['link_name'])
                pos_world = self.data.oMf[frame_id].translation
                gui_kinematics[s_name] = {
                    'pos': pos_world, # [x, y, z]
                    'rot': rot        # 3x3 matrix
                }

                gravity_scale = self._gravity_scaling(rot)
                
                raw = tactile_data[s_name]
                f_vec = np.array([raw['fx'], raw['fy'], raw['fz']], dtype=float)
                
                features = module['processor'].process(f_vec, self.dt)
                features['shear_rate'][:2] *= gravity_scale

                # 构建 Link 力向量 (Z->Y)
                f_filtered = features['force_filtered']
                f_link_vec = np.zeros(3)
                f_link_vec[0] = f_filtered[0]
                f_link_vec[1] = f_filtered[2] 
                f_link_vec[2] = f_filtered[1] 
                
                # [修改] 存入 GUI 数据包
                gui_forces[s_name] = f_link_vec.tolist()

                # 发送给 ShearDebugger 示波器
                if s_name == self.debug_target_sensor: 
                    self.debugger.log(
                        sensor_name=s_name,
                        normal_force=features['normal_force'],
                        tangential_force=features['tangential_force'],
                        shear_rate=np.linalg.norm(features['shear_rate'][:2]),
                        slip_ratio=features['slip_ratio'],
                        pid_out=last_pid_out,
                        adm_vel=np.linalg.norm(module['admittance'].vel_error)
                    )

                if features['normal_force'] > 0.02:
                    if not module['contact_active']:
                        module['contact_active'] = True
                        module['pid'].reset()
                        module['admittance'].reset()
                    
                    finger_entries.append({
                        "name": s_name,
                        "features": features,
                        "f_control_link": f_link_vec,
                        "module": module,
                        "rotation": rot,
                        "jacobian": J
                    })
                else:
                    module['contact_active'] = False
                    module['last_pid_val'] = 0.0
            
            if not finger_entries: continue 
            
            # 4. 控制计算
            tangential_forces = np.array([e["features"]["tangential_force"] + 1e-3 for e in finger_entries])
            weights = tangential_forces / tangential_forces.sum()
            finger_dq_accum = np.zeros(len(self.current_q))
            
            for weight, entry in zip(weights, finger_entries):
                features = entry["features"]
                f_meas_link = entry["f_control_link"]
                module = entry["module"]
                rot = entry["rotation"]
                J = entry["jacobian"]
                
                shear_err = -features['shear_rate'][:2]
                f_tangent = module['pid'].update(shear_err, self.dt)
                module['last_pid_val'] = np.linalg.norm(f_tangent)
                
                f_normal = 2.0
                if features['slip_ratio'] > 0.5:
                    f_normal += (features['slip_ratio'] - 0.5) * 5.0
                
                f_cmd_local = np.array([f_tangent[0], f_normal, f_tangent[1]]) 
                rot_np = np.array(rot, copy=False)
                
                f_cmd_world = rot_np @ f_cmd_local
                f_meas_world = rot_np @ f_meas_link
                f_err = f_cmd_world - f_meas_world
                
                if np.linalg.norm(f_err) < 0.02: f_err[:] = 0.0
                    
                module['admittance'].update(f_err)
                dx = module['admittance'].vel_error * self.dt
                
                dq_sensor = self._solve_ik_underactuated(J, dx)
                finger_dq_accum += weight * dq_sensor

            total_dq += finger_dq_accum
            
        total_dq = np.clip(total_dq, -0.025, 0.025)
        target_q_deg = np.rad2deg(self.current_q + total_dq) 
        try:
            self.hand.set_joint_positions(target_q_deg.tolist())
        except Exception: pass

        # ==========================================================
        # [修改] 通过 multiprocessing Queue 发送数据给 GUI
        # ==========================================================
        if self.gui_queue and not self.gui_queue.full():
            try:
                # 构造数据包
                packet = {
                    'q': self.current_q,
                    'forces': gui_forces,
                    'kinematics': gui_kinematics
                }
                self.gui_queue.put(packet)
            except Exception:
                pass

    def run(self):
        self.running = True
        try:
            while self.running:
                start = time.time()
                self.step()
                time.sleep(max(0, self.dt - (time.time() - start)))
        except KeyboardInterrupt:
            self.hand.set_actuations([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.tactile.close()

if __name__ == "__main__":
    # 如果单独运行此文件，依然可以使用，只是没有 GUI 接收队列
    controller = ShearGraspController(str(URDF_PATH))
    controller.run()