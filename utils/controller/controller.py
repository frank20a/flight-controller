from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QTimer
from controller_ui import Ui_MainWindow
from functools import partial
import sys, serial


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.ser = serial.Serial('COM6', 115200, timeout=0.1)
        
        self.trpy = [0, 512, 512, 512]
        self.btn_states = [False, False]
        
        self.sliders = [ self.ui.thrust, self.ui.roll, self.ui.pitch, self.ui.yaw]
        
        self.ui.btn1.clicked.connect(partial(self.btn_clicked, 0))
        self.ui.btn2.clicked.connect(partial(self.btn_clicked, 1))
        self.ui.thrust.valueChanged.connect(partial(self.slider_changed, 0))
        self.ui.roll.sliderReleased.connect(lambda: self.ui.roll.setValue(512))
        self.ui.roll.valueChanged.connect(partial(self.slider_changed, 1))
        self.ui.pitch.sliderReleased.connect(lambda: self.ui.pitch.setValue(512))
        self.ui.pitch.valueChanged.connect(partial(self.slider_changed, 2))
        self.ui.yaw.sliderReleased.connect(lambda: self.ui.yaw.setValue(512))
        self.ui.yaw.valueChanged.connect(partial(self.slider_changed, 3))
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.callback)
        self.timer.start(1000 / 20)
        
    def slider_changed(self, slider, event=None):
        self.trpy[slider] = self.sliders[slider].value()
        
    def btn_clicked(self, btn, event=None):
        self.btn_states[btn] = not self.btn_states[btn]
    
    def callback(self, event=None):
        res = bytearray()
        res += (self.trpy[0] << 6).to_bytes(2, 'little')
        res += (self.trpy[1] << 6).to_bytes(2, 'little')
        res += (self.trpy[2] << 6 | self.btn_states[0] << 5 | self.btn_states[1] << 4).to_bytes(2, 'little')
        res += (self.trpy[3] << 6).to_bytes(2, 'little')
        res += bytes(4)
        
        self.ser.write(res)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()
