import serial
import time
import struct
import threading
import logging
import numpy as np  # 引入numpy方便计算均值
from typing import Dict, List, Optional

# ... (前面的协议常量定义保持不变) ...
# -------------------------- 协议常量定义 --------------------------
BAUDRATE = 921600
REQ_HEAD = b"\x55\xAA"
RESP_HEAD_GENERAL = b"\xAA\x55"
AUTO_PUSH_FRAME_HEAD = b"\xAA\x56"

# 寄存器地址
VERSION_REG = 0x0000
SENSOR_COMBO_REG = 0x0010
DATA_TYPE_REG = 0x0016
AUTO_PUSH_REG = 0x0017

# 功能码
FUNC_READ = 0x03
FUNC_WRITE = 0x10

# 传感器名称映射
ALL_SENSORS_MAP = [
    "Thumb-Prox", "Thumb-Mid", "Thumb-Tip", "Thumb-Nail",
    "Index-Prox", "Index-Mid", "Index-Tip", "Index-Nail",
    "Middle-Prox", "Middle-Mid", "Middle-Tip", "Middle-Nail",
    "Ring-Prox", "Ring-Mid", "Ring-Tip", "Ring-Nail",
    "Pinky-Prox", "Pinky-Mid", "Pinky-Tip", "Pinky-Nail",
    "Palm-1", "Palm-2", "Palm-3", "Palm-4",
    "Palm-5", "Palm-6", "Palm-7", "Palm-8"
]

class TactileSensor:
    def __init__(self, port: str = "/dev/ttyACM0"):
        self.port = port
        self.ser: Optional[serial.Serial] = None
        self.active_sensors: List[str] = []
        self.latest_data: Dict[str, Dict[str, int]] = {}
        
        # [新增] 存储零点偏移量: {'SensorName': [off_x, off_y, off_z]}
        self._offsets: Dict[str, np.ndarray] = {}
        self._is_tared = False
        
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        
        logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
        self.logger = logging.getLogger("TactileSensor")

        self._connect_and_initialize()

    def _connect_and_initialize(self):
        # ... (连接逻辑保持不变，除了初始化 offsets) ...
        try:
            self.logger.info(f"正在连接串口 {self.port} ...")
            self.ser = serial.Serial(self.port, baudrate=BAUDRATE, timeout=0.5)
            if not self.ser.is_open:
                self.ser.open()
            
            self._set_auto_push(False)
            self.ser.reset_input_buffer()
            time.sleep(0.2)
            
            self.active_sensors = self._detect_sensors()
            if not self.active_sensors:
                raise RuntimeError("未检测到任何传感器，请检查连接")
            
            with self._lock:
                for name in self.active_sensors:
                    self.latest_data[name] = {"fx": 0, "fy": 0, "fz": 0}
                    # [新增] 初始化偏移量为0
                    self._offsets[name] = np.zeros(3)

            self._configure_force_mode()
            self._set_auto_push(True)
            
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self.logger.info("触觉传感器初始化完成，后台接收中...")

        except Exception as e:
            self.logger.error(f"初始化失败: {e}")
            if self.ser and self.ser.is_open:
                self.ser.close()
            raise

    # [核心新增功能] 标定/去皮
    def tare(self, duration: float = 2.0):
        """
        进行零点标定。采集一段时间的数据计算平均值作为偏移量。
        :param duration: 采集时长(秒)，建议 1~2秒
        """
        self.logger.info(f"开始触觉传感器标定 (持续 {duration}秒)... 请勿触碰传感器！")
        
        start_time = time.time()
        samples = {name: [] for name in self.active_sensors}
        
        # 采集数据
        while time.time() - start_time < duration:
            # 直接读取当前瞬时数据（未去皮的）
            # 注意：这里我们还没设置 _is_tared，所以 get_latest_data 此时返回的是原始值
            # 但为了严谨，我们直接读 _raw_latest_data (需要稍微修改解析逻辑)
            # 或者更简单：在 get_latest_data 里根据标志位决定是否减去偏移
            # 这里我们直接用 parse 线程更新的 latest_data，因为 parse 线程会减去旧的 offset
            # 所以最好的做法是：先暂时清空 offset
            with self._lock:
                self._is_tared = False 
                for name in self.active_sensors:
                    self._offsets[name] = np.zeros(3)
            
            time.sleep(0.01) # 让数据流进来
            
            with self._lock:
                current = self.latest_data
                for name in self.active_sensors:
                    if name in current:
                        d = current[name]
                        samples[name].append([d['fx'], d['fy'], d['fz']])
            time.sleep(0.05) # 采样间隔

        # 计算均值
        with self._lock:
            for name, data_list in samples.items():
                if len(data_list) > 0:
                    arr = np.array(data_list)
                    mean_val = np.mean(arr, axis=0) # [mean_x, mean_y, mean_z]
                    self._offsets[name] = mean_val
                    self.logger.info(f"  {name} Offset: {mean_val.astype(int)}")
            self._is_tared = True
            
        self.logger.info("标定完成！后续数据将自动扣除此偏移量。")

    def get_latest_data(self) -> Dict[str, Dict[str, float]]:
        """
        获取去皮后的合力数据
        """
        with self._lock:
            # 此时 latest_data 已经被 _parse_packet 减去偏移量了（见下面修改）
            # 或者我们在解析层不做处理，在读取层做处理。
            # 为了效率，建议在解析层直接减掉。
            return self.latest_data.copy()

    # ... (close, calc_lrc, build_frame, send_recv, detect, config, set_push 保持不变) ...
    def close(self):
        """关闭设备和线程"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        
        if self.ser and self.ser.is_open:
            try:
                self._set_auto_push(False) # 尝试发送停止指令
            except:
                pass
            self.ser.close()
            self.logger.info("设备已关闭")

    def _calc_lrc(self, data: bytes) -> int:
        lrc_sum = 0
        for byte in data:
            lrc_sum = (lrc_sum + byte) & 0xFF
        return ((~lrc_sum) + 1) & 0xFF

    def _build_frame(self, func_code: int, reg_addr: int, data_len: int, write_data: bytes = b"") -> bytes:
        reg_bytes = reg_addr.to_bytes(2, "little")
        len_bytes = data_len.to_bytes(2, "little")
        body = REQ_HEAD + b"\x00" + func_code.to_bytes(1, "big") + reg_bytes + len_bytes + write_data
        lrc = self._calc_lrc(body)
        return body + lrc.to_bytes(1, "big")

    def _send_recv(self, frame: bytes, expect_head: bytes, timeout: float = 0.5) -> Optional[bytes]:
        """同步发送接收（仅用于初始化阶段）"""
        self.ser.write(frame)
        start_t = time.time()
        recv_buf = b""
        while time.time() - start_t < timeout:
            if self.ser.in_waiting:
                recv_buf += self.ser.read(self.ser.in_waiting)
                if expect_head in recv_buf and len(recv_buf) >= 8:
                    return recv_buf
            time.sleep(0.002)
        return None

    def _detect_sensors(self) -> List[str]:
        """读取寄存器判断连接了哪些传感器"""
        cmd = self._build_frame(FUNC_READ, SENSOR_COMBO_REG, 4)
        resp = self._send_recv(cmd, RESP_HEAD_GENERAL)
        
        sensors = []
        if resp and len(resp) >= 12: # Head(2)+...Data(4)...
            # 提取数据段 (通常在第8字节开始)
            data_segment = resp[8:12]
            current_map_idx = 0
            for byte_val in data_segment:
                for bit_idx in range(8):
                    if current_map_idx < len(ALL_SENSORS_MAP):
                        if (byte_val >> bit_idx) & 0x01:
                            sensors.append(ALL_SENSORS_MAP[current_map_idx])
                        current_map_idx += 1
        return sensors

    def _configure_force_mode(self):
        """设置数据类型为合力"""
        cmd = self._build_frame(FUNC_WRITE, DATA_TYPE_REG, 1, b"\x01")
        self._send_recv(cmd, RESP_HEAD_GENERAL)

    def _set_auto_push(self, enable: bool):
        """开关自动回传"""
        val = b"\x01" if enable else b"\x00"
        cmd = self._build_frame(FUNC_WRITE, AUTO_PUSH_REG, 1, val)
        self.ser.write(cmd)
        time.sleep(0.05)

    def _read_loop(self):
        """后台线程：持续读取串口流并解析"""
        buffer = b""
        while self._running and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    buffer += self.ser.read(self.ser.in_waiting)
                    
                    while AUTO_PUSH_FRAME_HEAD in buffer:
                        head_idx = buffer.find(AUTO_PUSH_FRAME_HEAD)
                        if head_idx > 0:
                            buffer = buffer[head_idx:]
                        
                        if len(buffer) < 5: 
                            break
                        
                        frame_valid_len = int.from_bytes(buffer[3:5], "little")
                        total_pkt_len = 6 + frame_valid_len + 1 
                        
                        if len(buffer) >= total_pkt_len:
                            packet = buffer[:total_pkt_len]
                            self._parse_packet(packet) # 调用解析
                            buffer = buffer[total_pkt_len:]
                        else:
                            break
                else:
                    time.sleep(0.001)
            except Exception as e:
                self.logger.error(f"读取线程错误: {e}")
                time.sleep(0.1)

    def _parse_packet(self, packet: bytes):
        """[核心修改] 解析单帧数据并应用偏移"""
        try:
            payload = packet[6:-1]
            sensor_count = len(self.active_sensors)
            expected_len = sensor_count * 6 
            
            if len(payload) >= expected_len:
                temp_data = {}
                for i, name in enumerate(self.active_sensors):
                    offset_idx = i * 6
                    raw_fx, raw_fy, raw_fz = struct.unpack_from('<hhh', payload, offset_idx)
                    
                    # [新增] 应用偏移量
                    off = self._offsets.get(name, np.zeros(3))
                    
                    # 减去偏置
                    real_fx = raw_fx - off[0]
                    real_fy = raw_fy - off[1]
                    real_fz = raw_fz - off[2]
                    
                    # 你可以在这里做死区处理，例如绝对值 < 2 的设为 0
                    if abs(real_fx) < 2: real_fx = 0
                    if abs(real_fy) < 2: real_fy = 0
                    if abs(real_fz) < 2: real_fz = 0

                    temp_data[name] = {"fx": real_fx, "fy": real_fy, "fz": real_fz}
                
                with self._lock:
                    self.latest_data = temp_data
        except Exception:
            pass