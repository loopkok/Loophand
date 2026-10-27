import time
import logging
import numpy as np
import pinocchio as pin
from collections import deque
from typing import Dict, List, Optional
import multiprocessing as mp
from enum import Enum

# 引入硬件驱动和控制算法
from aero_open_sdk.aero_hand import AeroHand
from loophand.driver.tactile_driver import TactileSensor
from loophand.controller.pid import BoundedPID
from loophand.controller.admittance import AdmittanceController
from loophand.viz.shear_debugger import ShearDebugger
from loophand.paths import URDF_PATH

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("ShearControl")

# ================= 1. 硬件配置 =================
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

# 状态定义
class GraspState(Enum):
    IDLE = -1
    PRE_SHAPE = 0      
    ACTIVE_GRASP = 1   

# ================= 2. 信号处理 =================
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

# ================= 3. 控制器主类 =================
class ShearGraspController:
    def __init__(self, urdf_path: str, port_hand="/dev/ttyACM1", port_tactile="/dev/ttyACM0", gui_queue=None, cmd_queue=None):
        logger.info("Init Hardware & Model...")
        self.hand = AeroHand(port=port_hand) 
        self.tactile = TactileSensor(port=port_tactile)

        time.sleep(1.0) 
        self.tactile.tare(duration=4.0)
        
        self.gui_queue = gui_queue
        self.cmd_queue = cmd_queue
        self.debugger = None 
        self.debug_target_sensor = "Index-Mid"
        
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        self.pin_active_indices = []
        self.user_active_indices = []
        
        self.thumb_abd_index = None    
        self.finger_map = {"Thumb": [], "Index": [], "Middle": [], "Ring": [], "Pinky": []}

        for name in ACTIVE_JOINT_NAMES:
            if self.model.existJointName(name):
                joint_id = self.model.getJointId(name)
                idx_v = self.model.idx_vs[joint_id]
                self.pin_active_indices.append(idx_v)
            
            if name in JOINT_ORDER_FULL:
                idx = JOINT_ORDER_FULL.index(name)
                self.user_active_indices.append(idx)
                
        for idx, name in enumerate(JOINT_ORDER_FULL):
            if "abd" in name: 
                if "thumb" in name: self.thumb_abd_index = idx
                continue
            if "thumb" in name:  self.finger_map["Thumb"].append(idx)
            elif "index" in name: self.finger_map["Index"].append(idx)
            elif "middle" in name: self.finger_map["Middle"].append(idx)
            elif "ring" in name:   self.finger_map["Ring"].append(idx)
            elif "pinky" in name:  self.finger_map["Pinky"].append(idx)

        self.dt = 0.01
        self._gravity_world = np.array([0.0, -1.0, 0.0], dtype=float)
        
        # [防尖峰内存] 必须保留这部分
        self.smooth_target_f = {name: 8.0 for name in SENSOR_TO_LINK_MAP}
        self.prev_dx = {name: np.zeros(3) for name in SENSOR_TO_LINK_MAP}
        
        self.modules = {}
        PID_PARAMS = (8.0, 0.5, 0.0, 5.0) 
        
        # 物理参数
        ADM_M = np.array([0.11, 0.11, 0.11]) 
        ADM_D = np.array([12.0, 12.0, 12.0])    
        ADM_K = np.array([0.0, 0.0, 0.0]) 

        for sensor_name, link_name in SENSOR_TO_LINK_MAP.items():
            self.modules[sensor_name] = {
                "link_name": link_name,
                "processor": SignalProcessor(),
                "pid": BoundedPID(*PID_PARAMS),
                "contact_active": False,
                "has_anchored": False, 
                "admittance": AdmittanceController(ADM_M, ADM_K, ADM_D, self.dt, max_vel=0.5, max_acc=10.0),
            }
        
        self.current_q = np.zeros(len(JOINT_ORDER_FULL))
        self.running = False
        self.state = GraspState.IDLE 
        
        self.params = {
            "mu_est": 0.4,           
            "safety_margin": 1.5,    
            "min_force": 8.0,        
            "approach_thumb_abd_speed": 0.03,
            "approach_thumb_speed": 0.01,
            "approach_other_speed": 0.03,
            "contact_thresh": 0.1,  
            "thumb_abd_target": 1.65,
            "pre_shape_tol": 0.05,
            "limit_speed": 0.06,
        }

    def _map_7_to_16(self, compact_deg: List[float]) -> np.ndarray:
        q_7 = np.deg2rad(np.array(compact_deg))
        q_16 = np.zeros(len(JOINT_ORDER_FULL))
        q_16[0] = q_7[0]; q_16[1] = q_7[1]; q_16[2] = q_7[2]; q_16[3] = q_7[2] * 1.0 
        val_idx = q_7[3]; q_16[4] = val_idx; q_16[5] = val_idx * 0.8; q_16[6] = val_idx * 0.6 
        val_mid = q_7[4]; q_16[7] = val_mid; q_16[8] = val_mid * 0.8; q_16[9] = val_mid * 0.6
        val_rng = q_7[5]; q_16[10] = val_rng; q_16[11] = val_rng * 0.8; q_16[12] = val_rng * 0.6
        val_pnk = q_7[6]; q_16[13] = val_pnk; q_16[14] = val_pnk * 0.8; q_16[15] = val_pnk * 0.6
        return np.array(q_16, dtype=np.float64)

    def _update_kinematics(self, q_user: np.ndarray, link_name: str):
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
        
        current_pos = np.array(self.data.oMf[frame_id].translation, copy=False)
        
        if "proximal" in link_name: local_offset = np.array([0.0, 0.0, 0.025]) 
        else: local_offset = np.array([0.0, 0.0, 0.0])
        
        r_world = rot @ local_offset
        current_pos_sensor = current_pos + r_world
        
        rx, ry, rz = r_world
        skew_r = np.array([[0, -rz, ry], [rz, 0, -rx], [-ry, rx, 0]])
        J_lin_sensor = J_lin_origin - skew_r @ J_ang
        return rot, J_lin_sensor, current_pos_sensor

    def _gravity_scaling(self, rotation_ws: np.ndarray) -> np.ndarray:
        def axis_gain(axis):
            axis_unit = axis / max(np.linalg.norm(axis), 1e-9)
            cos_theta = np.clip(abs(float(axis_unit.dot(self._gravity_world))), 0.0, 1.0)
            theta = float(np.arccos(cos_theta))
            return float(np.clip(1.0 - (2.0 / np.pi) * theta, 0.0, 1.0))
        return np.array([axis_gain(rotation_ws[:, 0]), axis_gain(rotation_ws[:, 1])], dtype=float)

    def _solve_ik_underactuated(self, J_full: np.ndarray, dx: np.ndarray):
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
        # ==========================================================
        # 1. 指令处理 & 调试器更新
        # ==========================================================
        if self.cmd_queue and not self.cmd_queue.empty():
            try:
                msg = self.cmd_queue.get(block=False)
                if msg == "OPEN_TUNER" and self.debugger is None:
                    self.debugger = ShearDebugger()
                elif msg == "START_GRASP":
                    logger.info(">>> [CMD] 收到指令：启动抓取 (IDLE -> PRE_SHAPE) <<<")
                    self.state = GraspState.PRE_SHAPE
            except: pass

        if self.debugger:
            new_params = self.debugger.get_updated_params()
            if new_params:
                self.debug_target_sensor = new_params.get('target', "Index-Mid")

        # ==========================================================
        # 2. 读取硬件
        # ==========================================================
        try:
            joint_deg_7 = self.hand.get_joint_positions_compact()
        except Exception: return
        if joint_deg_7 is None: return
        self.current_q = self._map_7_to_16(joint_deg_7)
        if np.isnan(self.current_q).any(): return
        
        tactile_data = self.tactile.get_latest_data()    
        total_dq = np.zeros(len(self.current_q)) # 初始化为0
        
        gui_forces = {}
        gui_kinematics = {} 
        
        for s_name in self.modules:
             if s_name in tactile_data:
                 raw = tactile_data[s_name]
                 gui_forces[s_name] = [raw['fx'], -raw['fz'], raw['fy']]

        # ==========================================================
        # 3. 状态机逻辑
        # ==========================================================
        
        # --- 状态 A: IDLE ---
        if self.state == GraspState.IDLE:
            if int(time.time()) % 5 == 0 and int(time.time()*10)%10 == 0:
                 print(f"[STATE] System IDLE... Waiting for START_GRASP command.")
            if self.gui_queue and not self.gui_queue.full():
                try:
                    for frame_id in range(self.model.nframes):
                        name = self.model.frames[frame_id].name
                        if "link" in name:
                            M = self.data.oMf[frame_id]
                            gui_kinematics[name] = {'pos': M.translation, 'rot': M.rotation}
                    self.gui_queue.put({'q': self.current_q.copy(), 'forces': gui_forces, 
                                        'kinematics': gui_kinematics, 'state': self.state.name})
                except: pass
            return

        # 计算运动学
        for frame_id in range(self.model.nframes):
            name = self.model.frames[frame_id].name
            if "link" in name:
                M = self.data.oMf[frame_id]
                gui_kinematics[name] = {'pos': M.translation, 'rot': M.rotation}

        # --- 状态 B: PRE_SHAPE ---
        if self.state == GraspState.PRE_SHAPE:
            if self.thumb_abd_index is None:
                self.state = GraspState.ACTIVE_GRASP
            else:
                current_val = self.current_q[self.thumb_abd_index]
                target_val = self.params['thumb_abd_target']
                diff = target_val - current_val
                if abs(diff) < self.params['pre_shape_tol']:
                    logger.info("预姿态到位 -> 进入独立控制")
                    self.state = GraspState.ACTIVE_GRASP
                else:
                    step_move = np.clip(diff, -self.params['approach_thumb_abd_speed'], self.params['approach_thumb_abd_speed'])
                    total_dq[self.thumb_abd_index] += step_move
            
            self._apply_and_send(total_dq, 0.5)
            return

        # --- 状态 C: ACTIVE_GRASP (防尖峰核心逻辑) ---
        if self.state == GraspState.ACTIVE_GRASP:
            
            all_finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
            ADM_DIRECTION = -1.0 
            
            for finger_name in all_finger_names:
                related_sensors = [s for s in SENSOR_TO_LINK_MAP.keys() if finger_name in s]
                finger_in_contact = False
                active_sensor_entries = []
                
                # --- 4.1 遍历传感器判断接触 ---
                for s_name in related_sensors:
                    if s_name not in tactile_data: continue
                    module = self.modules[s_name]
                    
                    rot, J, pos = self._update_kinematics(self.current_q, module['link_name'])
                    gravity_scale = self._gravity_scaling(rot)
                    raw = tactile_data[s_name]
                    features = module['processor'].process(np.array([raw['fx'], raw['fy'], raw['fz']]), self.dt)
                    features['shear_rate'][:2] *= gravity_scale

                    f_filtered_sensor = features['force_filtered']
                    f_control_link = np.zeros(3)
                    f_control_link[0] = f_filtered_sensor[0] 
                    f_control_link[1] = f_filtered_sensor[2]
                    f_control_link[2] = f_filtered_sensor[1] 

                    f_viz = f_control_link.copy(); f_viz[1] = -f_viz[1] 
                    gui_forces[s_name] = f_viz.tolist()
                    
                    # [关键修改] 迟滞逻辑 (Hysteresis) 消除 Reset 尖峰
                    THRESH_ON = self.params['contact_thresh'] # 0.1
                    THRESH_OFF = 0.03 # 远小于 THRESH_ON，只有力几乎消失才允许断开
                    
                    is_touching = False
                    if module['contact_active']:
                        if features['normal_force'] > THRESH_OFF: is_touching = True
                    else:
                        if features['normal_force'] > THRESH_ON: is_touching = True
                    
                    if is_touching:
                        finger_in_contact = True
                        module['contact_active'] = True
                        active_sensor_entries.append({
                            "name": s_name, "features": features, "f_control_link": f_control_link,
                            "module": module, "rotation": rot, "jacobian": J, "pos": pos
                        })
                    else:
                        module['contact_active'] = False
                        module['has_anchored'] = False 
                        module['pid'].reset()
                        module['admittance'].reset() 
                        self.prev_dx[s_name] = np.zeros(3) # 复位平滑器

                # --- 4.2 独立控制决策 ---
                
                # [分支 A] 未接触
                if not finger_in_contact:
                    indices = self.finger_map.get(finger_name, [])
                    speed = self.params['approach_thumb_speed'] if finger_name == "Thumb" else self.params['approach_other_speed']
                    # [闭合逻辑 - 可选]
                    # for idx in indices: total_dq[idx] += speed 
                    pass
                    
                # [分支 B] 已接触：锚定导纳控制
                else:
                    tangential_forces = np.array([e["features"]["tangential_force"] + 1e-3 for e in active_sensor_entries])
                    weights = tangential_forces / tangential_forces.sum()
                    finger_dq_accum = np.zeros(len(self.current_q))
                    
                    for weight, entry in zip(weights, active_sensor_entries):
                        features = entry["features"]
                        module = entry["module"]
                        J = entry["jacobian"]
                        
                        # 1. 锚定
                        if not module.get('has_anchored', False):
                            module['admittance'].reset()
                            module['has_anchored'] = True
                            print(f"!!! [{entry['name']}] ANCHOR SET (F={features['normal_force']:.2f})")
                        
                        # 2. 目标力计算 (保留原逻辑+滤波)
                        SENSOR_NOISE_FLOOR = 0.2  
                        f_min_safe = max(self.params['min_force'], SENSOR_NOISE_FLOOR)
                        
                        f_physics_need = (features['tangential_force'] / self.params['mu_est']) * self.params['safety_margin']                  
                        f_target_raw = max(f_min_safe, f_physics_need)
                        
                        if features['slip_ratio'] > self.params['mu_est']:
                            f_target_raw += (features['slip_ratio'] - self.params['mu_est']) * 5.0
                        
                        # [滤波] 平滑目标力
                        alpha_target = 0.1 
                        f_target_smoothed = alpha_target * f_target_raw + (1.0 - alpha_target) * self.smooth_target_f[entry['name']]
                        self.smooth_target_f[entry['name']] = f_target_smoothed
                        f_target = f_target_smoothed
                        
                        # PID
                        shear_err = -features['shear_rate'][:2]
                        f_tangent = module['pid'].update(shear_err, self.dt)
                        module['last_pid_val'] = np.linalg.norm(f_tangent)

                        if self.debugger and entry["name"] == self.debug_target_sensor:
                            self.debugger.log(
                                sensor_name=entry["name"],
                                normal_force=features['normal_force'],
                                tangential_force=features['tangential_force'],
                                shear_rate=np.linalg.norm(features['shear_rate'][:2]),
                                slip_ratio=features['slip_ratio'],
                                pid_out=module['last_pid_val'],
                                adm_vel=-module['admittance'].vel_error[1]
                            )
                        
                        # 3. 构造指令
                        f_cmd_local = np.array([f_tangent[0], f_target, f_tangent[1]])
                        rot_np = np.array(entry["rotation"], copy=False)
                        f_meas_world = rot_np @ entry["f_control_link"]
                        f_cmd_world = rot_np @ f_cmd_local
                        
                        force_error = f_cmd_world - f_meas_world
                        
                        # [限幅] 物理保护
                        force_error = np.clip(force_error, -2.0, 2.0)
                        if np.linalg.norm(force_error) < 0.02: force_error[:] = 0.0
                        
                        # 4. 导纳计算
                        module['admittance'].update(force_error)
                        dx_raw = module['admittance'].vel_error * self.dt * ADM_DIRECTION
                        print(f"module['admittance'].vel_error: {module['admittance'].vel_error}")
                        
                        # [滤波] 速度输出平滑
                        alpha_vel = 0.4
                        dx_smoothed = alpha_vel * dx_raw + (1.0 - alpha_vel) * self.prev_dx[entry['name']]
                        self.prev_dx[entry['name']] = dx_smoothed
                        
                        dx = dx_smoothed

                        dq_sensor = self._solve_ik_underactuated(J, dx)
                        finger_dq_accum += weight * dq_sensor
                    
                    total_dq += finger_dq_accum

            self._apply_and_send(total_dq, self.params['limit_speed'])
        
        # 6. 发送 GUI 数据
        if self.gui_queue and not self.gui_queue.full():
            try:
                clean_kinematics = {n: {'pos': d['pos'].copy(), 'rot': d['rot'].copy()} for n, d in gui_kinematics.items()}
                self.gui_queue.put({'q': self.current_q.copy(), 'forces': gui_forces, 
                                    'kinematics': clean_kinematics, 'state': self.state.name})
            except: pass

    def _apply_and_send(self, total_dq, limit_val):
        total_dq = np.clip(total_dq, -limit_val, limit_val)
        target_q_16 = np.rad2deg(self.current_q + total_dq) 
        try:
            target_actuations = target_q_16[self.user_active_indices]
            self.hand.set_joint_positions(target_actuations.tolist())
        except Exception: pass

    def run(self):
        self.running = True
        try:
            while self.running:
                start = time.time()
                self.step()
                time.sleep(max(0, self.dt - (time.time() - start)))
        except KeyboardInterrupt:
            self.hand.set_actuations([0.0]*7); self.tactile.close()

if __name__ == "__main__":
    controller = ShearGraspController(str(URDF_PATH))
    controller.run()