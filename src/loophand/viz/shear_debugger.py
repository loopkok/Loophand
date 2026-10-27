import multiprocessing as mp
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore, QtWidgets
import sys
import time

# 传感器列表 (必须与主程序中的 keys 保持一致)
SENSOR_LIST = [
    "Thumb-Tip",
    "Index-Tip", "Index-Mid",
    "Middle-Tip", "Middle-Mid",
    "Ring-Tip", "Ring-Mid",
    "Pinky-Tip", "Pinky-Mid"
]

class ShearDebugger:
    def __init__(self):
        self.data_queue = mp.Queue(maxsize=1000)
        self.param_queue = mp.Queue(maxsize=10)
        self.process = mp.Process(target=self._run_ui, args=(self.data_queue, self.param_queue))
        self.process.daemon = True
        self.process.start()

    def log(self, sensor_name, normal_force, tangential_force, shear_rate, slip_ratio, pid_out, adm_vel):
        if not self.data_queue.full():
            self.data_queue.put({
                'name': sensor_name,
                'Fn': normal_force, 'Ft': tangential_force,
                'dF': shear_rate, 'slip': slip_ratio,
                'pid': np.linalg.norm(pid_out), 'adm_v': np.linalg.norm(adm_vel),
                'time': time.time()
            })

    def get_updated_params(self):
        params = None
        while not self.param_queue.empty():
            params = self.param_queue.get()
        return params

    def close(self):
        self.process.terminate()

    @staticmethod
    def _run_ui(data_q, param_q):
        app = pg.mkQApp("Shear Controller Debugger")
        win = QtWidgets.QMainWindow()
        win.setWindowTitle("Shear & Slip Controller Tuning Scope")
        win.resize(1200, 800)

        cw = QtWidgets.QWidget()
        win.setCentralWidget(cw)
        layout = QtWidgets.QHBoxLayout()
        cw.setLayout(layout)

        # --- 1. 左侧绘图区 ---
        plot_layout = pg.GraphicsLayoutWidget()
        layout.addWidget(plot_layout, stretch=3)

        pen_r = pg.mkPen('r', width=2); pen_g = pg.mkPen('g', width=2)
        pen_y = pg.mkPen('y', width=2); pen_c = pg.mkPen('c', width=2)
        pen_m = pg.mkPen('m', width=2); pen_w = pg.mkPen('w', width=2)

        p1 = plot_layout.addPlot(title="Forces")
        p1.setLabel('left', 'Force', units='N'); p1.showGrid(x=True, y=True); p1.addLegend()
        curve_Fn = p1.plot(pen=pen_r, name='Normal Force (Fn)')
        curve_Ft = p1.plot(pen=pen_g, name='Tangential Force (Ft)')
        plot_layout.nextRow()

        p2 = plot_layout.addPlot(title="Shear & PID")
        p2.setLabel('left', 'Mag'); p2.showGrid(x=True, y=True); p2.addLegend()
        curve_dF = p2.plot(pen=pen_y, name='Shear Rate (dF/dt)')
        curve_PID = p2.plot(pen=pen_c, name='PID Output')
        plot_layout.nextRow()

        p3 = plot_layout.addPlot(title="Slip & Admittance")
        p3.showGrid(x=True, y=True); p3.addLegend()
        curve_Slip = p3.plot(pen=pen_m, name='Slip Ratio')
        curve_Adm = p3.plot(pen=pen_w, name='Adm Velocity')
        
        try: dash_style = QtCore.Qt.PenStyle.DashLine
        except AttributeError: dash_style = QtCore.Qt.DashLine
        threshold_line = pg.InfiniteLine(pos=0.5, angle=0, pen=pg.mkPen('b', style=dash_style), label='Slip Thresh')
        p3.addItem(threshold_line)

        # --- 2. 右侧参数区 ---
        ctrl_panel = QtWidgets.QWidget()
        ctrl_layout = QtWidgets.QVBoxLayout()
        ctrl_panel.setLayout(ctrl_layout)
        layout.addWidget(ctrl_panel, stretch=1)

        # [新增] 传感器选择下拉框
        ctrl_layout.addWidget(QtWidgets.QLabel("<h3>Select Target Sensor</h3>"))
        combo_sensor = QtWidgets.QComboBox()
        combo_sensor.addItems(SENSOR_LIST)
        # 默认选 Index-Mid
        idx = combo_sensor.findText("Index-Mid")
        if idx >= 0: combo_sensor.setCurrentIndex(idx)
        ctrl_layout.addWidget(combo_sensor)
        
        ctrl_layout.addSpacing(10)

        def create_param_row(label_text, min_v, max_v, init_v, step=0.01):
            row_widget = QtWidgets.QWidget()
            row_layout = QtWidgets.QHBoxLayout()
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_widget.setLayout(row_layout)

            lbl = QtWidgets.QLabel(label_text)
            lbl.setFixedWidth(60)
            
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(min_v, max_v); spin.setSingleStep(step); spin.setValue(init_v); spin.setDecimals(3)
            
            try: orientation = QtCore.Qt.Orientation.Horizontal
            except AttributeError: orientation = QtCore.Qt.Horizontal
            slider = QtWidgets.QSlider(orientation)
            
            scale = 100.0 / step if step > 0 else 100.0
            slider.setRange(int(min_v * scale), int(max_v * scale))
            slider.setValue(int(init_v * scale))

            def on_spin_change(val):
                slider.blockSignals(True)
                slider.setValue(int(val * scale))
                slider.blockSignals(False)

            def on_slider_change(val):
                spin.blockSignals(True)
                spin.setValue(val / scale)
                spin.blockSignals(False)

            spin.valueChanged.connect(on_spin_change)
            slider.valueChanged.connect(on_slider_change)

            row_layout.addWidget(lbl); row_layout.addWidget(spin); row_layout.addWidget(slider)
            ctrl_layout.addWidget(row_widget)
            return spin

        ctrl_layout.addWidget(QtWidgets.QLabel("<h3>PID Parameters</h3>"))
        sb_kp = create_param_row("Kp", 0, 100, 8.0)
        sb_ki = create_param_row("Ki", 0, 20, 0.5)
        sb_kd = create_param_row("Kd", 0, 10, 0.0)

        ctrl_layout.addSpacing(20)
        ctrl_layout.addWidget(QtWidgets.QLabel("<h3>Admittance</h3>"))
        sb_m = create_param_row("Mass", 0.001, 0.2, 0.02, step=0.001)
        sb_d = create_param_row("Damping", 0.1, 100, 20.0, step=0.1)
        
        ctrl_layout.addStretch()

        history_len = 500
        data_buffer = {
            'Fn': np.zeros(history_len), 'Ft': np.zeros(history_len),
            'dF': np.zeros(history_len), 'slip': np.zeros(history_len),
            'pid': np.zeros(history_len), 'adm': np.zeros(history_len)
        }

        def update():
            # [核心修改] 发送 target sensor 名称
            if param_q.empty():
                param_q.put({
                    'target': combo_sensor.currentText(), # 发送当前下拉框选中的名字
                    'kp': sb_kp.value(), 'ki': sb_ki.value(), 'kd': sb_kd.value(),
                    'm': sb_m.value(), 'd': sb_d.value()
                })

            while not data_q.empty():
                item = data_q.get()
                # 只有当收到的数据名字 和 当前下拉框名字一致时，才绘图
                # 这样可以防止切换瞬间的跳变，虽然Controller端已经做了过滤
                if item['name'] == combo_sensor.currentText():
                    data_buffer['Fn'][:-1] = data_buffer['Fn'][1:]; data_buffer['Fn'][-1] = item['Fn']
                    data_buffer['Ft'][:-1] = data_buffer['Ft'][1:]; data_buffer['Ft'][-1] = item['Ft']
                    data_buffer['dF'][:-1] = data_buffer['dF'][1:]; data_buffer['dF'][-1] = item['dF']
                    data_buffer['slip'][:-1] = data_buffer['slip'][1:]; data_buffer['slip'][-1] = item['slip']
                    data_buffer['pid'][:-1] = data_buffer['pid'][1:]; data_buffer['pid'][-1] = item['pid']
                    data_buffer['adm'][:-1] = data_buffer['adm'][1:]; data_buffer['adm'][-1] = item['adm_v']

            curve_Fn.setData(data_buffer['Fn'])
            curve_Ft.setData(data_buffer['Ft'])
            curve_dF.setData(data_buffer['dF'])
            curve_PID.setData(data_buffer['pid'])
            curve_Slip.setData(data_buffer['slip'])
            curve_Adm.setData(data_buffer['adm'])

        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(30) 

        win.show()
        if hasattr(app, 'exec'): sys.exit(app.exec())
        else: sys.exit(app.exec_())