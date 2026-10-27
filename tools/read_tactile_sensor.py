import os
import time
import serial
import serial.tools.list_ports
from typing import List, Optional, Dict
from pathlib import Path
import logging

# -------------------------- 设备协议配置（与GEN3传感器匹配）--------------------------
AUTO_PUSH_REG = 0x0017               # 自动回传控制寄存器（1=开启，0=关闭）
AUTO_PUSH_FRAME_HEAD = b"\xAA\x56"   # 自动回传数据帧头
VERSION_REG = 0x0000                 # 版本号寄存器地址
VERSION_DATA_LEN = 0x000F            # 版本号数据长度（15字节）
DATA_TYPE_REG = 0x0016               # 数据类型组合寄存器
BAUDRATE = 921600                    # 高速通信波特率
TIMEOUT_CMD = 1.0                    # 指令响应超时（秒）
TIMEOUT_AUTO_PUSH = 0.05             # 自动回传监听超时（秒）

# 帧结构常量
REQ_HEAD = b"\x55\xAA"               # 请求帧头（主机→传感器）
RESP_HEAD_GENERAL = b"\xAA\x55"      # 普通响应帧头（传感器→主机）
RESP_HEAD_AUTO_PUSH = b"\xAA\x56"    # 自动回传相关响应帧头
RESERVED = b"\x00"                   # 预留字段
FUNC_READ = 0x03                     # 读寄存器功能码
FUNC_WRITE = 0x10                    # 写寄存器功能码

# 指令示例（用于校验）
CMD_EXAMPLE_AUTO_PUSH = "55AA00101700010001D8"  # 开启自动推送示例指令
CMD_EXAMPLE_VERSION = "55AA000300000F00EF"      # 读版本号示例指令

# -------------------------- 日志配置 --------------------------
LOG_DIR = Path(os.environ.get("LOOPHAND_LOG_DIR", Path(__file__).resolve().parents[1] / "logs"))
LOG_DIR.mkdir(parents=True, exist_ok=True)

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(module)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler(LOG_DIR / "sensor_comm.log", encoding="utf-8"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def send_hex_cmd(ser: serial.Serial, hex_cmd: str) -> bool:
    """发送16进制指令"""
    try:
        cmd_bytes = bytes.fromhex(hex_cmd)
        ser.flushOutput()
        sent_len = ser.write(cmd_bytes)
        if sent_len != len(cmd_bytes):
            logger.error(f"指令发送不完整：应发{len(cmd_bytes)}字节，实发{sent_len}字节 | 指令：{hex_cmd}")
            return False
        logger.debug(f"指令发送成功：{hex_cmd}（{sent_len}字节）")
        return True
    except (serial.SerialException, ValueError, Exception) as e:
        logger.error(f"发送错误：{str(e)} | 指令：{hex_cmd}", exc_info=True)
        return False


def read_serial_data(ser: serial.Serial, timeout: float = TIMEOUT_CMD, expected_head: Optional[bytes] = None) -> Optional[bytes]:
    """读取串口数据（支持指定预期帧头）"""
    try:
        start_time = time.time()
        recv_data = b""
        while time.time() - start_time < timeout:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                recv_data += chunk
                logger.debug(f"接收数据片段：{chunk.hex()}（{len(chunk)}字节）")
                
                # 如果指定了预期帧头，检查是否已收到
                if expected_head and expected_head in recv_data:
                    # 从帧头开始截取数据
                    head_pos = recv_data.find(expected_head)
                    recv_data = recv_data[head_pos:]
                    break  # 找到预期帧头后退出等待
                    
                start_time = time.time()  # 重置超时计时器
                time.sleep(0.005)  # 等待可能的后续数据
            time.sleep(0.001)
        
        if recv_data:
            logger.debug(f"完整数据接收：{recv_data.hex()}（{len(recv_data)}字节）")
            return recv_data
        logger.warning(f"超时未接收数据（{timeout}秒）")
        return None
    except Exception as e:
        logger.error(f"读取错误：{str(e)}", exc_info=True)
        return None


def calc_lrc(data: bytes) -> int:
    """计算LRC校验（累加→取反→加1→低8位）"""
    try:
        lrc_sum = 0
        for byte in data:
            lrc_sum = (lrc_sum + byte) & 0xFF  # 8位累加防溢出
        lrc = ((~lrc_sum) + 1) & 0xFF         # 补码计算
        logger.debug(f"LRC计算：{data.hex()} → 0x{lrc:02X}")
        return lrc
    except Exception as e:
        logger.error(f"LRC计算错误：{str(e)}", exc_info=True)
        return 0


def build_request_frame(func_code: int, reg_addr: int, data_len: int, write_data: bytes = b"") -> Optional[str]:
    """构建请求帧（完全匹配协议：Head+预留+功能码+寄存器地址+数据长度+数据+LRC）"""
    try:
        # 寄存器地址和数据长度均采用小端模式（匹配协议要求）
        reg_addr_bytes = reg_addr.to_bytes(2, byteorder="little")
        data_len_bytes = data_len.to_bytes(2, byteorder="little")
        
        # 组装帧主体（不含LRC）
        frame_without_lrc = (
            REQ_HEAD + 
            RESERVED + 
            func_code.to_bytes(1, "big") + 
            reg_addr_bytes + 
            data_len_bytes + 
            write_data
        )
        
        # 计算LRC并补全帧
        lrc = calc_lrc(frame_without_lrc).to_bytes(1, "big")
        full_frame = frame_without_lrc + lrc
        frame_hex = full_frame.hex().upper()
        
        # 校验与示例指令一致性
        if func_code == FUNC_WRITE and reg_addr == AUTO_PUSH_REG and write_data == b"\x01" and data_len == 1:
            if frame_hex != CMD_EXAMPLE_AUTO_PUSH:
                logger.warning(f"开启自动推送指令与示例不符：生成{frame_hex}，示例{CMD_EXAMPLE_AUTO_PUSH}")
        if func_code == FUNC_READ and reg_addr == VERSION_REG and data_len == VERSION_DATA_LEN:
            if frame_hex != CMD_EXAMPLE_VERSION:
                logger.warning(f"读版本号指令与示例不符：生成{frame_hex}，示例{CMD_EXAMPLE_VERSION}")
        
        logger.debug(f"请求帧构建：{frame_hex}（{len(full_frame)}字节）")
        return frame_hex
    except Exception as e:
        logger.error(f"请求帧构建错误：{str(e)}", exc_info=True)
        return None


def parse_auto_response(response: bytes) -> Optional[Dict]:
    """解析自动回传相关响应（包括关闭指令响应），帧头为AA 56"""
    try:
        # 基础校验：最小帧长度
        if len(response) < 7:
            logger.error(f"自动回传响应帧过短：{len(response)}字节 | 数据：{response.hex()}")
            return None
        
        # 帧头校验（AA 56）
        if response[:2] != RESP_HEAD_AUTO_PUSH:
            logger.error(f"自动回传响应帧头错误：{response[:2].hex()} 期望{RESP_HEAD_AUTO_PUSH.hex()}")
            return None
        
        # 提取基础字段
        parsed = {
            "head": response[:2].hex(),
            "reserved": response[2],
            "valid_frame_len": int.from_bytes(response[3:5], "little"),  # 有效帧长度：小端
            "error_code": response[5],
            "valid_data": b"",
            "valid_data_len": 0,
            "lrc_valid": False,
            "lrc_calc": 0,
            "lrc_recv": response[-1] if len(response) >= 7 else 0
        }
        
        # 计算有效数据长度
        parsed["valid_data_len"] = parsed["valid_frame_len"] - 1
        
        # 提取有效数据
        if parsed["valid_data_len"] > 0:
            data_end_pos = 6 + parsed["valid_data_len"]
            if data_end_pos <= len(response) - 1:  # 预留LRC位置
                parsed["valid_data"] = response[6:data_end_pos]
            else:
                parsed["valid_data"] = response[6:-1]  # 截取到LRC前
                logger.warning(f"自动回传响应数据不完整：期望{parsed['valid_data_len']}字节，实际{len(parsed['valid_data'])}字节")
        
        # LRC校验
        if len(response) >= 6 + parsed["valid_data_len"] + 1:
            parsed["lrc_calc"] = calc_lrc(response[:-1])
            parsed["lrc_valid"] = (parsed["lrc_calc"] == parsed["lrc_recv"])
            if not parsed["lrc_valid"]:
                logger.warning(f"自动回传响应LRC校验失败：计算0x{parsed['lrc_calc']:02X}，实际0x{parsed['lrc_recv']:02X}")
        
        logger.debug(f"自动回传响应解析完成：{parsed}")
        return parsed
    except Exception as e:
        logger.error(f"自动回传响应解析错误：{str(e)} | 数据：{response.hex()}", exc_info=True)
        return None


def parse_response(response: bytes) -> Optional[Dict]:
    """解析普通设备响应帧（Head+预留+功能码+地址+数据长度+数据+LRC）"""
    try:
        # 基础校验：最小帧长度
        if len(response) < 8:  # 无数据时最小8字节
            logger.error(f"响应帧过短：{len(response)}字节 | 数据：{response.hex()}")
            return None
        
        # 帧头校验（普通响应为AA 55）
        if response[:2] != RESP_HEAD_GENERAL:
            logger.error(f"帧头不匹配：{response[:2].hex()} 期望{RESP_HEAD_GENERAL.hex()} | 数据：{response.hex()}")
            return None
        
        # 提取字段（所有多字节字段采用小端解析）
        parsed = {
            "is_error": False,
            "reserved": response[2],
            "func_code": response[3],
            "reg_addr": int.from_bytes(response[4:6], "little"),
            "data_len": int.from_bytes(response[6:8], "little"),
            "actual_data_len": len(response) - 9 if len(response) > 8 else 0,
            "data": b"",
            "lrc_valid": False,
            "lrc_calc": 0,
            "lrc_recv": response[-1] if len(response) >= 9 else 0
        }
        
        # 错误响应处理（功能码最高位为1表示错误）
        if (parsed["func_code"] & 0x80) != 0:
            parsed["is_error"] = True
            parsed["error_code"] = parsed["func_code"] & 0x7F
            logger.warning(f"设备错误：0x{parsed['error_code']:02X} | 地址0x{parsed['reg_addr']:04X}")
            return parsed
        
        # 功能码校验
        if parsed["func_code"] not in [FUNC_READ, FUNC_WRITE]:
            logger.error(f"无效功能码：0x{parsed['func_code']:02X}")
            return None
        
        # 提取有效数据
        if parsed["data_len"] > 0 and len(response) >= 8 + parsed["data_len"] + 1:
            parsed["data"] = response[8:8 + parsed["data_len"]]
            logger.debug(f"提取数据：{parsed['data'].hex()}（{len(parsed['data'])}字节）")
        elif parsed["data_len"] > 0:
            parsed["data"] = response[8:-1] if len(response) > 8 else b""
            logger.warning(f"数据不完整：期望{parsed['data_len']}字节，实际{len(parsed['data'])}字节")
        
        # LRC校验
        if len(response) >= 9:
            parsed["lrc_calc"] = calc_lrc(response[:-1])
            parsed["lrc_valid"] = (parsed["lrc_calc"] == parsed["lrc_recv"])
            if not parsed["lrc_valid"]:
                logger.warning(f"LRC校验失败：计算0x{parsed['lrc_calc']:02X}，实际0x{parsed['lrc_recv']:02X}")
        
        # 数据长度一致性校验
        if parsed["data_len"] != len(parsed["data"]):
            logger.warning(f"数据长度不匹配：期望{parsed['data_len']}，实际{len(parsed['data'])}")
        
        return parsed
    except Exception as e:
        logger.error(f"响应解析错误：{str(e)} | 数据：{response.hex()}", exc_info=True)
        return None


def read_register(ser: serial.Serial, reg_addr: int, read_len: int) -> Optional[bytes]:
    """读取寄存器数据（功能码0x03）"""
    if not (1 <= read_len <= 512):
        logger.error(f"无效读取长度：{read_len}字节（协议限制1-512字节）")
        return None
    
    read_frame = build_request_frame(FUNC_READ, reg_addr, read_len)
    if not read_frame:
        logger.error(f"构建读请求失败：0x{reg_addr:04X}，{read_len}字节")
        return None
    
    if not send_hex_cmd(ser, read_frame):
        logger.error(f"发送读请求失败")
        return None
    
    time.sleep(0.2)  # 预留响应时间
    response = read_serial_data(ser, TIMEOUT_CMD, RESP_HEAD_GENERAL)
    if not response:
        logger.error(f"未收到读响应")
        return None
    
    parsed = parse_response(response)
    if not parsed or parsed["is_error"] or parsed["func_code"] != FUNC_READ:
        logger.error(f"读操作失败")
        return None
    
    return parsed["data"]


def write_register(ser: serial.Serial, reg_addr: int, write_data: bytes, is_auto_push: bool = False) -> bool:
    """写入寄存器数据（功能码0x10），is_auto_push标识是否为自动回传相关操作"""
    write_len = len(write_data)
    if not (1 <= write_len <= 10):
        logger.error(f"无效写入长度：{write_len}字节（协议限制1-10字节）")
        return False
    
    write_frame = build_request_frame(FUNC_WRITE, reg_addr, write_len, write_data)
    if not write_frame:
        logger.error(f"构建写请求失败：0x{reg_addr:04X}")
        return False
    
    if not send_hex_cmd(ser, write_frame):
        logger.error(f"发送写请求失败")
        return False
    
    time.sleep(0.2)  # 预留响应时间
    
    # 根据是否为自动回传相关操作选择不同的帧头
    expected_head = RESP_HEAD_AUTO_PUSH if is_auto_push else RESP_HEAD_GENERAL
    response = read_serial_data(ser, TIMEOUT_CMD, expected_head)
    if not response:
        logger.error(f"未收到写响应")
        return False
    
    # 解析响应
    if is_auto_push:
        parsed = parse_auto_response(response)
        # 自动回传响应通过error_code判断是否成功（0表示成功）
        if not parsed or parsed["error_code"] != 0:
            logger.error(f"自动回传相关写操作失败，错误码：0x{parsed['error_code']:02X}" if parsed else "自动回传相关写操作响应解析失败")
            return False
    else:
        parsed = parse_response(response)
        if not parsed or parsed["is_error"] or parsed["func_code"] != FUNC_WRITE:
            logger.error(f"写操作失败")
            return False
        
        # 验证写入状态（返回数据为0表示成功）
        if len(parsed["data"]) > 0:
            write_status = int.from_bytes(parsed["data"], "little")
            if write_status != 0:
                logger.error(f"写入状态错误：0x{write_status:02X}（0表示成功）")
                return False
    
    return True


def disable_auto_push(ser: serial.Serial) -> bool:
    """关闭自动回传功能（写入0x0017寄存器为0x00）"""
    try:
        # 直接构建并发送关闭指令，不等待响应
        disable_cmd = build_request_frame(FUNC_WRITE, AUTO_PUSH_REG, 1, b"\x00")
        if not disable_cmd:
            logger.error("构建关闭自动回传指令失败")
            return False
            
        # 发送指令但不验证响应
        if send_hex_cmd(ser, disable_cmd):
            logger.info("关闭自动回传指令已发送")
            # 短暂延迟确保指令被接收
            time.sleep(0.1)
            return True
        return False
    except Exception as e:
        logger.error(f"关闭自动回传失败：{str(e)}", exc_info=True)
        return False


def enable_auto_push(ser: serial.Serial) -> bool:
    """开启自动回传功能（写入0x0017寄存器为0x01）"""
    return write_register(ser, AUTO_PUSH_REG, b"\x01", is_auto_push=True)

def parse_auto_push_data(data: bytes, expected_length: int = 0) -> Optional[Dict]:
    """解析自动回传数据帧（AA56头+预留+有效帧长度+总错误码+有效数据+LRC）"""
    try:
        # 基础校验：最小帧长度（Head2+预留1+有效帧长度2+总错误码1+LRC1=7字节）
        if len(data) < 7:
            logger.error(f"自动回传帧过短：{len(data)}字节 | 数据：{data.hex()}")
            return None
        
        # 帧头校验
        if data[:2] != AUTO_PUSH_FRAME_HEAD:
            logger.error(f"自动回传帧头错误：{data[:2].hex()} 期望{AUTO_PUSH_FRAME_HEAD.hex()} | 数据：{data.hex()}")
            return None
        
        # 提取基础字段
        valid_frame_len = int.from_bytes(data[3:5], "little")  # 有效帧长度：小端
        error_code = data[5]
        
        # 计算完整帧长度（包括帧头、预留、长度字段、错误码、数据、LRC）
        # 有效帧长度 = 有效数据长度 + 1（错误码）
        valid_data_len = valid_frame_len - 1
        total_frame_length = 6 + valid_frame_len  # 头2+预留1+长度2+错误码1+数据(valid_data_len)+LRC1
        
        # 检查数据是否完整
        if len(data) < total_frame_length:
            logger.warning(f"数据帧不完整：期望{total_frame_length}字节，实际{len(data)}字节")
            # 即使不完整，也尝试解析已有数据
            total_frame_length = len(data)
        
        # 如果数据长度超过完整帧长度，说明有多个帧粘连，只取第一个完整帧
        actual_data = data[:total_frame_length] if len(data) > total_frame_length else data
        
        # 重新提取字段
        parsed = {
            "head": actual_data[:2].hex(),
            "reserved": actual_data[2],
            "valid_frame_len": valid_frame_len,
            "error_code": error_code,
            "valid_data": b"",
            "valid_data_len": valid_data_len,
            "expected_data_len": expected_length,
            "length_match": False,
            "lrc_valid": False,
            "lrc_calc": 0,
            "lrc_recv": actual_data[total_frame_length - 1] if total_frame_length <= len(actual_data) else 0,
            "total_frame_length": total_frame_length,
            "actual_received_length": len(data)
        }
        
        # 提取有效数据
        if valid_data_len > 0:
            data_start_pos = 6  # 头2+预留1+长度2+错误码1
            data_end_pos = data_start_pos + valid_data_len
            if data_end_pos <= len(actual_data) - 1:  # 预留LRC位置
                parsed["valid_data"] = actual_data[data_start_pos:data_end_pos]
            else:
                # 数据不完整，能取多少取多少
                parsed["valid_data"] = actual_data[data_start_pos:min(data_end_pos, len(actual_data)-1)]
                logger.warning(f"自动回传数据不完整：期望{valid_data_len}字节，实际{len(parsed['valid_data'])}字节")
        
        # 校验数据长度是否符合预期配置
        if expected_length > 0:
            parsed["length_match"] = (valid_data_len == expected_length)
            if not parsed["length_match"]:
                logger.warning(f"数据长度与配置不符：实际{valid_data_len}字节，期望{expected_length}字节")
        
        # LRC校验 - 计算从帧头到LRC之前的所有字节
        if total_frame_length <= len(actual_data):
            lrc_data_range = actual_data[:total_frame_length - 1]  # 不包括LRC本身
            parsed["lrc_calc"] = calc_lrc(lrc_data_range)
            parsed["lrc_valid"] = (parsed["lrc_calc"] == parsed["lrc_recv"])
            
            if not parsed["lrc_valid"]:
                logger.warning(f"自动回传LRC校验失败：计算0x{parsed['lrc_calc']:02X}，实际0x{parsed['lrc_recv']:02X}")
                logger.debug(f"LRC计算数据范围：{lrc_data_range.hex()}（{len(lrc_data_range)}字节）")
                logger.debug(f"完整数据帧：{actual_data.hex()}（{len(actual_data)}字节）")
        else:
            logger.warning("数据帧长度不足，无法进行LRC校验")
        
        return parsed
    except Exception as e:
        logger.error(f"自动回传解析错误：{str(e)} | 数据：{data.hex()}", exc_info=True)
        return None

def get_device_version(ser: serial.Serial) -> Optional[str]:
    """获取设备版本号"""
    version_data = read_register(ser, VERSION_REG, VERSION_DATA_LEN)
    if version_data:
        try:
            ascii_version = version_data.decode("ascii", errors="ignore").strip()
            return f"ASCII: {ascii_version} | 16进制: {version_data.hex().upper()}"
        except:
            return f"16进制: {version_data.hex().upper()}"
    return None


def monitor_auto_push_detailed(ser: serial.Serial, config: Dict, duration: Optional[float] = None) -> None:
    """使用详细解析的自动回传监听"""
    start_time = time.time()
    frame_count = 0
    
    print("\n===== 自动回传数据详细解析 =====")
    print(f"按Ctrl+C停止")
    print("-" * 80)
    
    try:
        while True:
            if duration and time.time() - start_time > duration:
                break
                
            push_data = read_serial_data(ser, TIMEOUT_AUTO_PUSH, AUTO_PUSH_FRAME_HEAD)
            if push_data:
                frame_count += 1
                parsed = parse_auto_push_data_detailed(push_data, config)
                
                if parsed:
                    print(f"\n[{time.strftime('%H:%M:%S')}] 帧#{frame_count}")
                    print(f"错误码: 0x{parsed['error_code']:02X} | 数据长度: {parsed['valid_data_len']}字节")
                    
                    # 显示传感器数据
                    if 'sensor_data' in parsed:
                        sensor_data = parsed['sensor_data']
                        
                        if 'force' in sensor_data and sensor_data['force']:
                            print("合力数据:")
                            for module, forces in sensor_data['force'].items():
                                print(f"  {module}: Fx={forces['fx']:6d}, Fy={forces['fy']:6d}, Fz={forces['fz']:6d}")
                        
                        if 'distributed' in sensor_data and sensor_data['distributed']:
                            print("分布力数据概要:")
                            for module, dist_data in sensor_data['distributed'].items():
                                if dist_data['points']:
                                    first_point = dist_data['points'][0]
                                    print(f"  {module}: {dist_data['point_count']}个点, 示例: Fx={first_point['fx']:4d}, Fy={first_point['fy']:4d}, Fz={first_point['fz']:4d}")
                    
                    print("-" * 80)
                    
    except KeyboardInterrupt:
        print(f"\n监听结束 - 总共接收 {frame_count} 帧")
    except Exception as e:
        logger.error(f"监听错误: {str(e)}")
        print(f"\n监听错误: {str(e)}")

def read_sensor_configuration(ser: serial.Serial) -> Dict:
    """读取传感器配置信息"""
    config = {}
    
    try:
        # 读取传感器模组组合 (0x0010-0x0013)
        print("读取传感器模组组合...")
        sensor_comb_data = read_register(ser, 0x0010, 4)
        if sensor_comb_data and len(sensor_comb_data) == 4:
            config['sensor_combination'] = {
                'byte0': sensor_comb_data[0],
                'byte1': sensor_comb_data[1], 
                'byte2': sensor_comb_data[2],
                'byte3': sensor_comb_data[3]
            }
            print(f"传感器模组组合: {sensor_comb_data.hex().upper()}")
            
            # 解析哪些模组已连接
            module_names = [
                "大拇指近节", "大拇指中节", "大拇指指尖", "大拇指指甲",
                "食指近节", "食指中节", "食指指尖", "食指指甲",
                "中指近节", "中指中节", "中指指尖", "中指指甲", 
                "无名指近节", "无名指中节", "无名指指尖", "无名指指甲",
                "小拇指近节", "小拇指中节", "小拇指指尖", "小拇指指甲",
                "掌心1", "掌心2", "掌心3", "掌心4", "掌心5", "掌心6", "掌心7", "掌心8"
            ]
            
            connected_modules = []
            module_index = 0
            for byte_idx in range(4):
                byte_val = sensor_comb_data[byte_idx]
                for bit_idx in range(8):
                    if module_index < len(module_names) and (byte_val & (1 << bit_idx)) != 0:
                        connected_modules.append(module_names[module_index])
                    module_index += 1
            
            if connected_modules:
                print(f"已连接的模组: {', '.join(connected_modules)}")
            else:
                print("未检测到连接的传感器模组")
        else:
            print("读取传感器模组组合失败")
            return None
        
        # 读取数据类型组合 (0x0016)
        print("读取数据类型组合...")
        data_type_data = read_register(ser, 0x0016, 1)
        if data_type_data and len(data_type_data) == 1:
            config['data_type'] = data_type_data[0]
            has_force = (data_type_data[0] & 0x01) != 0
            has_distributed = (data_type_data[0] & 0x02) != 0
            print(f"数据类型组合: 0x{data_type_data[0]:02X} (合力: {'是' if has_force else '否'}, 分布力: {'是' if has_distributed else '否'})")
        else:
            print("读取数据类型组合失败，使用默认值0x03")
            config['data_type'] = 0x03
        
        # 读取分布力点数配置 (0x0030-0x0067)
        print("读取分布力点数配置...")
        point_config_data = read_register(ser, 0x0030, 56)
        if point_config_data and len(point_config_data) == 56:
            config['point_config'] = point_config_data
            
            # 解析各模组的分布力点数
            module_names_short = [
                "拇近", "拇中", "拇尖", "拇指",
                "食近", "食中", "食尖", "食指",
                "中近", "中中", "中尖", "中指",
                "无近", "无中", "无尖", "无指", 
                "小近", "小中", "小尖", "小指",
                "掌1", "掌2", "掌3", "掌4", "掌5", "掌6", "掌7", "掌8"
            ]
            
            point_info = []
            for i in range(28):
                point_addr = i * 2
                if point_addr + 2 <= len(point_config_data):
                    point_count = int.from_bytes(point_config_data[point_addr:point_addr+2], 'little')
                    if point_count > 0:
                        point_info.append(f"{module_names_short[i]}:{point_count}点")
            
            if point_info:
                print(f"分布力点数: {', '.join(point_info)}")
            else:
                print("所有模组分布力点数为0")
        else:
            print("读取分布力点数配置失败")
        
        return config
        
    except Exception as e:
        logger.error(f"读取传感器配置错误: {str(e)}")
        print(f"读取传感器配置错误: {str(e)}")
        return None

def monitor_auto_push_detailed(ser: serial.Serial, config: Dict, duration: Optional[float] = None) -> None:
    """使用详细解析的自动回传监听"""
    start_time = time.time()
    frame_count = 0
    valid_frame_count = 0
    
    print("\n" + "="*60)
    print("开始接收自动回传数据（详细解析）")
    print("按Ctrl+C停止监听")
    print("="*60)
    
    try:
        while True:
            # 检查监听时长
            if duration and time.time() - start_time > duration:
                print(f"\n监听超时（{duration}秒）")
                break
            
            # 读取自动回传数据
            push_data = read_serial_data(ser, TIMEOUT_AUTO_PUSH, AUTO_PUSH_FRAME_HEAD)
            if push_data:
                frame_count += 1
                parsed = parse_auto_push_data_detailed(push_data, config)
                
                if parsed:
                    valid_frame_count += 1
                    
                    # 显示基本信息
                    print(f"\n[{time.strftime('%H:%M:%S')}] 帧#{frame_count}")
                    print(f"错误码: 0x{parsed['error_code']:02X} | 数据长度: {parsed['valid_data_len']}字节")
                    
                    # 显示传感器数据
                    if 'sensor_data' in parsed:
                        sensor_data = parsed['sensor_data']
                        
                        # 显示合力数据
                        if 'force' in sensor_data and sensor_data['force']:
                            print("【合力数据】")
                            for module, forces in sensor_data['force'].items():
                                # 转换为实际物理值（根据协议，可能需要乘以系数）
                                fx_n = forces['fx'] * 0.1  # 示例系数，根据实际调整
                                fy_n = forces['fy'] * 0.1
                                fz_n = forces['fz'] * 0.1
                                print(f"  {module:8s}: Fx={forces['fx']:6d} ({fx_n:5.1f}N), Fy={forces['fy']:6d} ({fy_n:5.1f}N), Fz={forces['fz']:6d} ({fz_n:5.1f}N)")
                        
                        # 显示分布力数据概要
                        if 'distributed' in sensor_data and sensor_data['distributed']:
                            print("【分布力数据概要】")
                            for module, dist_data in sensor_data['distributed'].items():
                                if dist_data['points'] and len(dist_data['points']) > 0:
                                    # 显示前3个点的数据作为示例
                                    sample_points = dist_data['points'][:3]
                                    sample_text = " | ".join([f"P{i+1}:({p['fx']:4d},{p['fy']:4d},{p['fz']:4d})" 
                                                            for i, p in enumerate(sample_points)])
                                    if len(dist_data['points']) > 3:
                                        sample_text += f" ... 共{dist_data['point_count']}点"
                                    print(f"  {module:8s}: {sample_text}")
                    
                    print("-" * 60)
                    
                    # 每10帧显示一次统计信息
                    if frame_count % 10 == 0:
                        success_rate = (valid_frame_count / frame_count) * 100
                        print(f"\n[统计] 总帧数: {frame_count}, 有效帧: {valid_frame_count}, 解析成功率: {success_rate:.1f}%")
                        print("-" * 60)
                else:
                    print(f"[{time.strftime('%H:%M:%S')}] 帧#{frame_count} - 解析失败")
                    print(f"  原始数据长度: {len(push_data)}字节")
                    print("-" * 60)
                    
    except KeyboardInterrupt:
        success_rate = (valid_frame_count / frame_count) * 100 if frame_count > 0 else 0
        print(f"\n监听结束 - 总帧数: {frame_count}, 有效帧: {valid_frame_count}, 解析成功率: {success_rate:.1f}%")
    except Exception as e:
        logger.error(f"监听错误：{str(e)}", exc_info=True)
        print(f"\n监听错误：{str(e)}")

def parse_auto_push_data_detailed(data: bytes, config: Dict) -> Optional[Dict]:
    """使用改进解析的自动回传数据解析"""
    try:
        if len(data) < 7:
            return None
        
        if data[:2] != AUTO_PUSH_FRAME_HEAD:
            return None
        
        # 基础解析
        valid_frame_len = int.from_bytes(data[3:5], "little")
        error_code = data[5]
        valid_data_len = valid_frame_len - 1
        
        if len(data) < 6 + valid_data_len + 1:
            logger.warning(f"数据不完整，期望{6+valid_data_len+1}字节，实际{len(data)}字节")
            return None
        
        valid_data = data[6:6+valid_data_len]
        
        parsed = {
            'head': data[:2].hex(),
            'valid_frame_len': valid_frame_len,
            'error_code': error_code,
            'valid_data_len': valid_data_len,
            'valid_data': valid_data,
            'sensor_data': {}
        }
        
        print(f"\n调试: 开始解析数据帧，有效数据长度: {valid_data_len}字节")
        print(f"调试: 完整数据: {valid_data.hex().upper()}")
        
        # 根据配置解析数据内容
        data_type = config.get('data_type', 0x03)
        has_force = (data_type & 0x01) != 0
        has_distributed = (data_type & 0x02) != 0
        
        # 解析合力数据（如果启用）
        if has_force:
            force_data = parse_force_data_improved(valid_data, config)
            if force_data:
                parsed['sensor_data']['force'] = force_data
        
        # 解析分布力数据（如果启用）
        if has_distributed:
            distributed_data = parse_distributed_force_data_improved(
                valid_data, config, 
                parsed['sensor_data'].get('force', {})
            )
            if distributed_data:
                parsed['sensor_data']['distributed'] = distributed_data
        
        return parsed
    except Exception as e:
        logger.error(f"详细解析错误: {str(e)}", exc_info=True)
        return None

def parse_force_data_improved(data: bytes, config: Dict) -> Dict:
    """改进的合力数据解析，适应串联情况"""
    force_data = {}
    
    # 检查传感器模组组合
    if 'sensor_combination' not in config:
        return force_data
    
    sensor_comb = config['sensor_combination']
    
    # 模组顺序：大拇指近节、中节、指尖、指甲，食指近节...
    module_names = [
        "大拇指近节", "大拇指中节", "大拇指指尖", "大拇指指甲",
        "食指近节", "食指中节", "食指指尖", "食指指甲",
        "中指近节", "中指中节", "中指指尖", "中指指甲",
        "无名指近节", "无名指中节", "无名指指尖", "无名指指甲",
        "小拇指近节", "小拇指中节", "小拇指指尖", "小拇指指甲",
        "掌心1", "掌心2", "掌心3", "掌心4", "掌心5", "掌心6", "掌心7", "掌心8"
    ]
    
    offset = 0
    module_index = 0
    
    # 调试信息：显示连接的模组
    connected_modules = []
    for byte_idx in range(4):
        byte_val = sensor_comb[f'byte{byte_idx}']
        for bit_idx in range(8):
            if module_index < len(module_names) and (byte_val & (1 << bit_idx)) != 0:
                connected_modules.append(module_names[module_index])
            module_index += 1
    
    print(f"调试: 连接的模组: {connected_modules}")
    
    # 重新开始遍历
    module_index = 0
    for byte_idx in range(4):
        byte_val = sensor_comb[f'byte{byte_idx}']
        for bit_idx in range(8):
            if module_index < len(module_names) and (byte_val & (1 << bit_idx)) != 0:
                # 该模组有连接，解析其合力数据
                if offset + 6 <= len(data):
                    fx = int.from_bytes(data[offset:offset+2], 'little', signed=True)
                    fy = int.from_bytes(data[offset+2:offset+4], 'little', signed=True) 
                    fz = int.from_bytes(data[offset+4:offset+6], 'little', signed=False)
                    
                    force_data[module_names[module_index]] = {
                        'fx': fx,
                        'fy': fy, 
                        'fz': fz,
                        'offset': offset  # 记录偏移量用于调试
                    }
                    
                    print(f"调试: {module_names[module_index]} 合力数据在偏移量 {offset}: Fx={fx}, Fy={fy}, Fz={fz}")
                    offset += 6
                else:
                    print(f"警告: {module_names[module_index]} 合力数据超出范围")
            module_index += 1
    
    return force_data

def parse_distributed_force_data_improved(data: bytes, config: Dict, force_data: Dict) -> Dict:
    """改进的分布力数据解析，使用合力数据的偏移信息"""
    distributed_data = {}
    
    # 计算合力数据总长度
    force_data_length = 0
    for module_name, force_info in force_data.items():
        if 'offset' in force_info:
            force_data_length = max(force_data_length, force_info['offset'] + 6)
    
    print(f"调试: 合力数据总长度: {force_data_length} 字节")
    
    offset = force_data_length  # 分布力数据在合力数据之后
    
    # 解析分布力数据
    if 'point_config' in config and 'sensor_combination' in config:
        point_config = config['point_config']
        sensor_comb = config['sensor_combination']
        
        module_names = [
            "大拇指近节", "大拇指中节", "大拇指指尖", "大拇指指甲",
            "食指近节", "食指中节", "食指指尖", "食指指甲",
            "中指近节", "中指中节", "中指指尖", "中指指甲",
            "无名指近节", "无名指中节", "无名指指尖", "无名指指甲",
            "小拇指近节", "小拇指中节", "小拇指指尖", "小拇指指甲",
            "掌心1", "掌心2", "掌心3", "掌心4", "掌心5", "掌心6", "掌心7", "掌心8"
        ]
        
        module_index = 0
        for byte_idx in range(4):
            byte_val = sensor_comb[f'byte{byte_idx}']
            for bit_idx in range(8):
                if module_index < len(module_names) and (byte_val & (1 << bit_idx)) != 0:
                    # 该模组有连接，获取其分布力点数
                    point_addr = module_index * 2
                    if point_addr + 2 <= len(point_config):
                        point_count = int.from_bytes(point_config[point_addr:point_addr+2], 'little')
                        
                        # 每个点3字节（Fx, Fy, Fz各1字节）
                        data_size = point_count * 3
                        
                        if offset + data_size <= len(data):
                            points = []
                            non_zero_count = 0
                            
                            for i in range(point_count):
                                point_offset = offset + i * 3
                                if point_offset + 3 > len(data):
                                    break
                                    
                                fx_byte = data[point_offset]
                                fy_byte = data[point_offset+1]
                                fz_byte = data[point_offset+2]
                                
                                fx = fx_byte - 256 if fx_byte >= 128 else fx_byte
                                fy = fy_byte - 256 if fy_byte >= 128 else fy_byte
                                fz = fz_byte
                                
                                # 检查是否非零
                                if fx != 0 or fy != 0 or fz != 0:
                                    non_zero_count += 1
                                
                                points.append({
                                    'fx': fx,
                                    'fy': fy,
                                    'fz': fz,
                                    'raw_bytes': f"{fx_byte:02X}{fy_byte:02X}{fz_byte:02X}"
                                })
                            
                            distributed_data[module_names[module_index]] = {
                                'point_count': point_count,
                                'non_zero_points': non_zero_count,
                                'points': points,
                                'offset': offset
                            }
                            
                            print(f"调试: {module_names[module_index]} 分布力数据在偏移量 {offset}: {point_count}点, {non_zero_count}个非零点")
                            
                            offset += data_size
                        else:
                            print(f"警告: {module_names[module_index]} 分布力数据超出范围")
                module_index += 1
    
    return distributed_data

def check_sensor_configuration(ser: serial.Serial):
    """详细检查传感器配置"""
    print("\n=== 详细传感器配置检查 ===")
    
    # 读取传感器模组组合
    sensor_comb = read_register(ser, 0x0010, 4)
    if sensor_comb:
        print(f"传感器模组组合 (0x0010-0x0013): {sensor_comb.hex().upper()}")
        
        # 解析每个bit代表的模组
        module_names = [
            "大拇指近节", "大拇指中节", "大拇指指尖", "大拇指指甲",
            "食指近节", "食指中节", "食指指尖", "食指指甲",
            "中指近节", "中指中节", "中指指尖", "中指指甲",
            "无名指近节", "无名指中节", "无名指指尖", "无名指指甲",
            "小拇指近节", "小拇指中节", "小拇指指尖", "小拇指指甲",
            "掌心1", "掌心2", "掌心3", "掌心4", "掌心5", "掌心6", "掌心7", "掌心8"
        ]
        
        connected = []
        for byte_idx in range(4):
            for bit_idx in range(8):
                module_idx = byte_idx * 8 + bit_idx
                if module_idx < len(module_names):
                    if sensor_comb[byte_idx] & (1 << bit_idx):
                        connected.append(module_names[module_idx])
        
        print(f"检测到的连接模组: {connected}")
    
    # 读取数据类型组合
    data_type = read_register(ser, 0x0016, 1)
    if data_type:
        dt = data_type[0]
        print(f"数据类型组合 (0x0016): 0x{dt:02X}")
        print(f"  合力使能: {'是' if dt & 0x01 else '否'}")
        print(f"  分布力使能: {'是' if dt & 0x02 else '否'}")
    
    # 读取分布力点数配置
    points_config = read_register(ser, 0x0030, 56)
    if points_config:
        print("各模组分布力点数:")
        module_names_short = ["拇近", "拇中", "拇尖", "拇指", "食近", "食中", "食尖", "食指", 
                             "中近", "中中", "中尖", "中指", "无近", "无中", "无尖", "无指",
                             "小近", "小中", "小尖", "小指", "掌1", "掌2", "掌3", "掌4", 
                             "掌5", "掌6", "掌7", "掌8"]
        
        for i in range(28):
            point_count = int.from_bytes(points_config[i*2:i*2+2], 'little')
            if point_count > 0:
                print(f"  {module_names_short[i]}: {point_count}点")
    
    print("="*50)

def validate_sensor_readings(ser: serial.Serial):
    """验证传感器读数，直接读取寄存器确认数据"""
    print("\n=== 传感器读数验证 ===")
    
    # 读取食指中节合力 (地址0x0506)
    middle_force = read_register(ser, 0x0506, 6)
    if middle_force and len(middle_force) == 6:
        fx = int.from_bytes(middle_force[0:2], 'little', signed=True)
        fy = int.from_bytes(middle_force[2:4], 'little', signed=True)
        fz = int.from_bytes(middle_force[4:6], 'little', signed=False)
        print(f"直接读取-食指中节合力: Fx={fx}, Fy={fy}, Fz={fz}")
    
    # 读取食指指尖合力 (地址0x050C)
    tip_force = read_register(ser, 0x050C, 6)
    if tip_force and len(tip_force) == 6:
        fx = int.from_bytes(tip_force[0:2], 'little', signed=True)
        fy = int.from_bytes(tip_force[2:4], 'little', signed=True)
        fz = int.from_bytes(tip_force[4:6], 'little', signed=False)
        print(f"直接读取-食指指尖合力: Fx={fx}, Fy={fy}, Fz={fz}")
    
    # 读取分布力配置
    points_config = read_register(ser, 0x0030, 56)
    if points_config:
        # 食指中节点数 (第5个模组，索引4)
        middle_points = int.from_bytes(points_config[8:10], 'little')  # 4*2=8
        # 食指指尖点数 (第7个模组，索引6)
        tip_points = int.from_bytes(points_config[12:14], 'little')    # 6*2=12
        print(f"分布力点数配置: 中节={middle_points}点, 指尖={tip_points}点")


def main():
    logger.info("=== GEN3触觉传感器程序启动 ===")
    
    # 扫描可用串口
    available_ports = list(serial.tools.list_ports.comports())
    if not available_ports:
        logger.error("无可用串口")
        print("错误：未找到可用串口")
        return
    
    # 选择串口
    print("\n可用串口：")
    for i, port in enumerate(available_ports, 1):
        print(f"  {i}. {port.device} - {port.description}")
    
    try:
        choice = int(input(f"\n选择串口（1-{len(available_ports)}）: "))
        selected_port = available_ports[choice - 1].device
    except (ValueError, IndexError):
        print("输入无效，选择第一个串口")
        selected_port = available_ports[0].device
    
    # 初始化串口
    ser = None
    try:
        ser = serial.Serial(
            port=selected_port,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            write_timeout=0.5,
            inter_byte_timeout=0.001
        )
        
        if not ser.is_open:
            ser.open()
        print(f"\n连接串口：{ser.name}（波特率{BAUDRATE}）")
        
        # 清空缓冲区
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)
        
        # 发送版本号指令
        print("\n发送版本号指令...")
        version_cmd = "55AA000300000F00EF"
        if send_hex_cmd(ser, version_cmd):
            print("版本号指令发送成功，等待响应...")
            response = read_serial_data(ser, TIMEOUT_CMD, RESP_HEAD_GENERAL)
            if response:
                print(f"收到版本号响应：{response.hex().upper()}")
                # 尝试解析版本号
                parsed = parse_response(response)
                if parsed and not parsed["is_error"]:
                    try:
                        ascii_version = parsed["data"].decode("ascii", errors="ignore").strip()
                        print(f"解析版本号：ASCII: {ascii_version}")
                    except:
                        print(f"解析版本号：16进制: {parsed['data'].hex().upper()}")
            else:
                print("未收到版本号响应")
        else:
            print("版本号指令发送失败")

        time.sleep(0.5)

        # 读取传感器配置信息
        print("\n读取传感器配置信息...")
        config = read_sensor_configuration(ser)
        
        # 详细检查传感器配置
        check_sensor_configuration(ser)
        
        # 验证传感器读数
        validate_sensor_readings(ser)
        
        if not config:
            print("读取传感器配置失败，使用默认配置继续")
            config = {
                'data_type': 0x03,  # 默认同时包含合力和分布力
                'sensor_combination': {
                    'byte0': 0x00,
                    'byte1': 0x00, 
                    'byte2': 0x00,
                    'byte3': 0x00
                }
            }
        
        time.sleep(0.5)

        # 先关闭可能的自动回传
        print("\n确保自动回传已关闭...")
        disable_auto_push(ser)
        time.sleep(0.5)
        
        # 清空缓冲区
        ser.reset_input_buffer()
        time.sleep(0.1)
        
        # 发送开启自动回传指令
        print("\n发送开启自动回传指令...")
        if enable_auto_push(ser):
            print("自动回传开启成功，开始监听数据...")
            # 短暂延迟后开始监听
            time.sleep(0.5)
            monitor_auto_push_detailed(ser, config)
        else:
            print("自动回传开启失败，尝试直接监听...")
            # 即使开启失败也尝试监听
            monitor_auto_push_detailed(ser, config)
    
    except serial.SerialException as e:
        logger.error(f"串口错误：{str(e)}")
        print(f"\n串口错误：{str(e)}")
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        logger.error(f"程序错误：{str(e)}", exc_info=True)
        print(f"\n程序错误：{str(e)}")
    finally:
        if ser and ser.is_open:
            print("\n发送关闭自动回传指令...")
            disable_success = disable_auto_push(ser)
            if disable_success:
                print("关闭自动回传指令已发送")
            else:
                print("关闭自动回传指令发送失败")
            ser.close()
            print("串口已关闭")
    
    logger.info("=== 程序结束 ===")


if __name__ == "__main__":
    main()
    