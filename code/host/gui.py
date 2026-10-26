# gui.py — 实时遥测与在线调参上位机（PyQt5 + pyqtgraph）
# 运行: python gui.py COM5    （端口号按设备管理器实际修改）
# 依赖: pip install pyserial numpy PyQt5 pyqtgraph
import sys, struct
import serial
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
from parser import TelemParser, crc16


class SerialThread(QtCore.QThread):
    frame_ready = QtCore.pyqtSignal(object)

    def __init__(self, port):
        super().__init__()
        self.ser = serial.Serial(port, 921600, timeout=0.05)
        self.parser = TelemParser()
        self.running = True

    def run(self):
        while self.running:
            data = self.ser.read(256)
            if data:
                for frame in self.parser.feed(data):
                    self.frame_ready.emit(frame)

    def stop(self):
        self.running = False
        self.wait(1000)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, port):
        super().__init__()
        self.setWindowTitle("平衡车遥测调参 V1.0")
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # ---- 四通道实时曲线 ----
        self.plot = pg.GraphicsLayoutWidget(); layout.addWidget(self.plot, 6)
        panel = QtWidgets.QHBoxLayout(); layout.addLayout(panel, 1)
        self.data, self.curves, self.idx = {}, {}, 0
        for row, (name, unit, color) in enumerate([
                ("theta", "deg", "b"), ("v", "m/s", "g"),
                ("u", "N·m", "r"), ("bias", "deg/s", "m")]):
            pw = self.plot.addPlot(row=row, col=0, title=f"{name} ({unit})")
            pw.showGrid(x=True, y=True, alpha=0.3)
            self.curves[name] = pw.plot(pen=pg.mkPen(color, width=1))
            self.data[name] = ([], [])

        # ---- 参数下发面板 ----
        self.vref = QtWidgets.QDoubleSpinBox(); self.vref.setRange(-0.5, 0.5)
        self.vref.setSingleStep(0.05)
        btn = QtWidgets.QPushButton("下发 v_ref")
        btn.clicked.connect(self.send_params)
        for w in (QtWidgets.QLabel("v_ref (m/s)"), self.vref, btn):
            panel.addWidget(w)

        self.t = SerialThread(port)
        self.t.frame_ready.connect(self.on_frame)
        self.t.start()

    def on_frame(self, fr):
        th, thd, v, u, bias = fr
        self.idx += 1
        for key, val in (("theta", th), ("v", v), ("u", u), ("bias", bias)):
            xs, ys = self.data[key]
            xs.append(self.idx); ys.append(val)
            if len(xs) > 2000:               # 滚动窗口约 20s
                xs.pop(0); ys.pop(0)
            self.curves[key].setData(xs, ys)

    def send_params(self):
        payload = struct.pack("<f", self.vref.value())
        head = bytes([0xAA, 0x55, 0x02, len(payload), 0])
        crc = crc16(head[2:] + payload)
        self.t.ser.write(head + payload + bytes([crc & 0xFF, crc >> 8]))

    def closeEvent(self, e):
        self.t.stop(); e.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    win = MainWindow(port); win.show()
    sys.exit(app.exec_())
