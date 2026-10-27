import os
import sys

# 强制 PyQt5
os.environ["PYQTGRAPH_QT_LIB"] = "PyQt5"
os.environ["QT_API"] = "pyqt5"

import time
import warnings
import multiprocessing as mp
import numpy as np

warnings.filterwarnings("ignore")

try:
    import pyqtgraph as pg
    from PyQt5 import QtWidgets, QtCore, QtGui
    from loophand.viz.hand_viz_3d import HandVisualizer
    from loophand.paths import URDF_PATH, MESHES_DIR
except ImportError as e:
    print(f"环境缺失: {e}")
    sys.exit(1)

# 传感器列表
SENSOR_LIST = [
    "Thumb-Tip",
    "Index-Tip", "Index-Mid",
    "Middle-Tip", "Middle-Mid",
    "Ring-Tip", "Ring-Mid",
    "Pinky-Tip", "Pinky-Mid"
]

class MainGUI(QtWidgets.QMainWindow):
    def __init__(self, data_queue, cmd_queue):
        super().__init__()
        self.data_queue = data_queue
        self.cmd_queue = cmd_queue
        
        self.setWindowTitle("Aero Hand Control Center")
        self.resize(1600, 900)
        self.setStyleSheet("background-color: #121212; color: #EEE;")
        
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)

        # === 左侧 ===
        left_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(left_layout, stretch=5)
        
        self.viz3d = HandVisualizer(str(URDF_PATH), str(MESHES_DIR))
        left_layout.addWidget(self.viz3d)
        
        btn_layout = QtWidgets.QHBoxLayout()
        self.btn_tuner = QtWidgets.QPushButton("Open Parameter Tuner")
        self.btn_tuner.setFixedHeight(40)
        self.btn_tuner.setStyleSheet("background-color: #007ACC; font-weight: bold;")
        self.btn_tuner.clicked.connect(self.open_tuner)
        btn_layout.addWidget(self.btn_tuner)
        left_layout.addLayout(btn_layout)

        # === 右侧 ===
        right_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(right_layout, stretch=3)

        group_set = QtWidgets.QGroupBox("Settings")
        form = QtWidgets.QFormLayout()
        
        self.cb_hand = QtWidgets.QComboBox(); self.cb_hand.addItems(["/dev/ttyACM1", "/dev/ttyACM0", "COM3"])
        self.cb_tact = QtWidgets.QComboBox(); self.cb_tact.addItems(["/dev/ttyACM0", "/dev/ttyACM1", "COM4"])
        
        # [1. 系统启动按钮]
        self.btn_start = QtWidgets.QPushButton("1. START SYSTEM (Connect)")
        self.btn_start.setStyleSheet("background-color: #4CAF50; color: white; padding: 8px; font-weight: bold;")
        self.btn_start.clicked.connect(self.start_process)

        # [2. 新增：抓取启动按钮]
        self.btn_grasp = QtWidgets.QPushButton("2. ACTIVATE GRASP (Start Control)")
        self.btn_grasp.setStyleSheet("background-color: #E040FB; color: white; padding: 8px; font-weight: bold;")
        self.btn_grasp.clicked.connect(self.activate_grasp)
        self.btn_grasp.setEnabled(False) # 初始禁用，连接硬件后才开启
        
        form.addRow("Hand:", self.cb_hand)
        form.addRow("Sensor:", self.cb_tact)
        form.addRow(self.btn_start)
        form.addRow(self.btn_grasp) # 添加到布局
        group_set.setLayout(form)
        right_layout.addWidget(group_set)

        # --- 曲线选择区域 ---
        plot_layout = QtWidgets.QHBoxLayout()
        plot_label = QtWidgets.QLabel("Select Plot Sensor:")
        self.cb_plot_sensor = QtWidgets.QComboBox()
        self.cb_plot_sensor.addItems(SENSOR_LIST)
        self.cb_plot_sensor.setCurrentText("Index-Mid")
        self.cb_plot_sensor.currentIndexChanged.connect(self.on_sensor_changed)
        plot_layout.addWidget(plot_label)
        plot_layout.addWidget(self.cb_plot_sensor)
        right_layout.addLayout(plot_layout)

        self.plot_force = pg.PlotWidget(title="Tactile Force")
        self.plot_force.addLegend(); self.plot_force.showGrid(x=True, y=True)
        self.plot_force.setYRange(-1, 5) 
        self.c_fx = self.plot_force.plot(pen='r', name='X'); self.c_fy = self.plot_force.plot(pen='g', name='Y'); self.c_fz = self.plot_force.plot(pen='b', name='Z')
        right_layout.addWidget(self.plot_force)

        self.plot_joint = pg.PlotWidget(title="Joint Angles")
        self.plot_joint.showGrid(x=True, y=True)
        self.plot_joint.setYRange(-10, 100)
        self.curve_joints = [self.plot_joint.plot(pen=(i,5)) for i in range(5)]
        right_layout.addWidget(self.plot_joint)

        self.buf_force = {'x': np.zeros(200), 'y': np.zeros(200), 'z': np.zeros(200)}
        self.buf_joint = np.zeros((5, 200))
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(30)
        
        self.control_process = None

    def start_process(self):
        h_port = self.cb_hand.currentText()
        t_port = self.cb_tact.currentText()
        print(f"🚀 启动硬件连接... Hand:{h_port}, Tactile:{t_port}")
        
        self.control_process = mp.Process(
            target=run_backend, 
            args=(h_port, t_port, self.data_queue, self.cmd_queue)
        )
        self.control_process.start()
        
        # UI 状态更新
        self.btn_start.setEnabled(False)
        self.btn_start.setText("SYSTEM CONNECTED")
        self.btn_grasp.setEnabled(True) # 允许点击开始抓取

    def activate_grasp(self):
        """发送开始抓取指令"""
        print("⚡ 发送抓取指令...")
        self.cmd_queue.put("START_GRASP")
        self.btn_grasp.setText("GRASPING ACTIVE")
        self.btn_grasp.setStyleSheet("background-color: #555; color: #CCC;") # 变灰
        self.btn_grasp.setEnabled(False)

    def open_tuner(self):
        if self.cmd_queue: self.cmd_queue.put("OPEN_TUNER")

    def on_sensor_changed(self):
        self.buf_force['x'].fill(0)
        self.buf_force['y'].fill(0)
        self.buf_force['z'].fill(0)
        self.plot_force.setTitle(f"Tactile Force ({self.cb_plot_sensor.currentText()})")

    def update_ui(self):
        for _ in range(10):
            if self.data_queue.empty(): break
            try:
                data = self.data_queue.get(block=False)
                
                if 'q' in data: self.viz3d.update_robot_pose(data['q']) 
                if 'forces' in data:
                    for s_name, f_vec in data['forces'].items():
                        link = self._map_sensor_to_link(s_name)
                        if link: self.viz3d.update_force_arrow(s_name, link, np.array(f_vec))

                target = self.cb_plot_sensor.currentText()
                if 'forces' in data and target in data['forces']:
                    f = data['forces'][target]
                    self.buf_force['x'][:-1] = self.buf_force['x'][1:]; self.buf_force['x'][-1] = f[0]
                    self.buf_force['y'][:-1] = self.buf_force['y'][1:]; self.buf_force['y'][-1] = f[1]
                    self.buf_force['z'][:-1] = self.buf_force['z'][1:]; self.buf_force['z'][-1] = f[2]
                    self.c_fx.setData(self.buf_force['x']); self.c_fy.setData(self.buf_force['y']); self.c_fz.setData(self.buf_force['z'])
                
                if 'q' in data:
                    q = data['q']
                    for i in range(5):
                        self.buf_joint[i][:-1] = self.buf_joint[i][1:]; self.buf_joint[i][-1] = np.rad2deg(q[i])
                        self.curve_joints[i].setData(self.buf_joint[i])
            except: pass

    def _map_sensor_to_link(self, s_name):
        mapping = {
            "Thumb-Tip": "left_thumb_tip_link",
            "Index-Tip": "left_index_tip_link", "Index-Mid": "left_index_proximal_link",
            "Middle-Tip": "left_middle_tip_link", "Middle-Mid": "left_middle_proximal_link",
            "Ring-Tip": "left_ring_tip_link", "Ring-Mid": "left_ring_proximal_link",
            "Pinky-Tip": "left_pinky_tip_link", "Pinky-Mid": "left_pinky_proximal_link"
        }
        return mapping.get(s_name, "")

    def closeEvent(self, event):
        if self.control_process: self.control_process.terminate()
        event.accept()

def run_backend(hand_port, tact_port, gui_q, cmd_q):
    try:
        from loophand.controller.shear_grasp_controller import ShearGraspController
        ctrl = ShearGraspController(str(URDF_PATH), port_hand=hand_port, port_tactile=tact_port,
                                    gui_queue=gui_q, cmd_queue=cmd_q)
        ctrl.run()
    except Exception as e:
        print(f"❌ Backend Error: {e}")


def main():
    try:
        mp.set_start_method("spawn")
    except RuntimeError:
        pass
    q_data = mp.Queue(maxsize=1000)
    q_cmd = mp.Queue(maxsize=10)
    app = QtWidgets.QApplication(sys.argv)
    win = MainGUI(q_data, q_cmd)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()