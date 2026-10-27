import time
from aero_open_sdk.aero_hand import AeroHand


class HybirdController: 
    def __init__(self, hand: AeroHand, mode: str = "position_control", set_forces: list = None):
        self.hand = hand
        self.mode = mode
        self.position_kp = -22.0  # 位置控制的比例增益
        self.initial_positions = []
        self.current_history = []
        self.init_time = 0.0
        self.stable_time = 1.0  # 稳定时间，单位秒
        self.history_size = 5
        self.current_mode = mode  # 初始模式由参数决定
        # 期望传入长度为7的力矩列表；如果未提供则初始化为全0
        if set_forces is None:
            self.set_forces = [0] * 7
        else:
            if not isinstance(set_forces, (list, tuple)):
                raise TypeError("set_forces must be a list or tuple of length 7")
            if len(set_forces) != 7:
                raise ValueError("set_forces must have 7 elements (one per actuator)")
            self.set_forces = list(set_forces)
    
    def switch_mode(self):
        if time.time() - self.init_time < self.stable_time:
            return  # 稳定时间内不切换模式
        current = self.hand.get_actuator_currents()
        # 根据电流值切换控制模式
        self.current_history.append(current)
        if len(self.current_history) > self.history_size:
            self.current_history.pop(0)
        # 检查历史电流数据是否稳定
        if len(self.current_history) < self.history_size:
            return  # 历史数据不足，暂不切换模式
        avg_current = [0] * 7
        for i in range(7):
            avg_current[i] = sum(hist[i] for hist in self.current_history) / len(self.current_history)
        #现在这个电流得根据位置模式的角度增量以及物体硬度实际来调整，当位置模式角度增量较快的时候，碰到物体的电流会比较大
        if self.current_mode == "position_control" and (avg_current[1] < -400 or avg_current[3] < -750):
            print(f"切换到力矩控制模式: 电流={avg_current}")
            self.current_mode = "torque_control"


    def pose_initial(self):
        # time.sleep(1.5)  # 这里是为了拍摄手操作不过来，先等一会儿
        trajectory = [  
            # ([65.0, 5.0, 20.0, 25.0, 0.0, 0.0, 0.0], 0.5),  
            # ([65.0, 5.0, 20.0, 25.0, 0.0, 0.0, 0.0], 0.25),  
            ([75.0, 5.0, 10.0, 30.0, 0.0, 0.0, 0.0], 0.5),  
            ([75.0, 5.0, 10.0, 30.0, 0.0, 0.0, 0.0], 0.25),  
        ]  
        self.hand.run_trajectory(trajectory)  
        time.sleep(1.0) # 等待到位
        self.initial_positions = self.hand.get_actuations()
        self.init_time = time.time()
        print(f"初始位置已记录: {self.initial_positions}")

    def position_controller(self):
        actuator_positions = self.hand.get_actuations()
        position_currents = self.hand.get_actuator_currents()
        print(f"位置控制模式: 当前位置为 {actuator_positions}")
        print(f"位置控制模式: 当前电流为 {position_currents}")
        move_positions = list(actuator_positions)
        for i in range(7):
            if i == 1:
                move_positions[i] += 1.0
                continue
            if i == 3:
                move_positions[i] += 2.0
                continue
            else:
                move_positions[i] = self.initial_positions[i]
        self.hand.set_actuations(move_positions)
        time.sleep(0.01)  # 100HZ执行频率
    
    def torque_controller(self):
        torque_positions = self.hand.get_actuations()
        pisition_errors = torque_positions[0]-self.initial_positions[0]
        torque_adjustment = int(pisition_errors * self.position_kp)
        if torque_adjustment > 1000 or torque_adjustment < -1000:
            torque_adjustment = max(min(torque_adjustment, 1000), -1000)
        torque_list = self.set_forces.copy()
        torque_list[0] += torque_adjustment  # 根据位置误差调整第一个舵机的力矩
        print(f"力矩控制模式: 目标力矩为 {torque_list}")
        self.hand.ctrl_torque(torque_list)
        time.sleep(0.01)  # 100HZ执行频率

    def control_loop(self):

        self.pose_initial()
        while True:
            try:
                self.switch_mode()
                temps = self.hand.get_actuator_temperatures()
                # 如果任意舵机温度超过80度，停止并退出控制循环
                if any(t > 80 for t in temps):
                    print(f"高温警告: 温度列表={temps}，停止控制循环")
                    self.hand.ctrl_torque([0] * 7)
                else:
                    if self.current_mode == "position_control":
                        self.position_controller()
                    else:
                        self.torque_controller()
            except KeyboardInterrupt:
                print("控制循环终止")
                # 回到初始张开位置
                self.hand.set_actuations([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                break
            except Exception as e:
                print(f"控制循环错误: {e}")
                time.sleep(0.1)



if __name__ == "__main__":  
    hand = AeroHand("/dev/ttyACM1")
    set_forces = [0, 120, 80, 120, 0, 0, 0]    
    test_controller = HybirdController(hand, set_forces=set_forces)
    test_controller.control_loop()

