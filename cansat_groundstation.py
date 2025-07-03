import sys
import serial
import serial.tools.list_ports
import numpy as np
import cv2
import math
import time
from PyQt5.QtWidgets import (
    QApplication, QLabel, QVBoxLayout, QWidget, QHBoxLayout, QPushButton,
    QComboBox, QGroupBox, QGridLayout, QLineEdit, QTabWidget, QProgressBar, QFileDialog
)
from PyQt5.QtGui import QImage, QPixmap, QColor
from PyQt5.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import qdarkstyle

# --------- Utilidades ---------
def list_serial_ports():
    return [port.device for port in serial.tools.list_ports.comports()]

def get_status_color(connected):
    return "background-color: #4CAF50;" if connected else "background-color: #F44336;"

# --------- Main Window ---------
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CanSat Ground Station")
        self.serial = None
        self.connected = False
        self.ser_port = None
        self.log_file = None

        # --------- Barra superior: Puerto COM y conexión ---------
        self.port_combo = QComboBox()
        self.refresh_ports()
        self.connect_btn = QPushButton("Conectar")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.status_label = QLabel("Desconectado")
        self.status_label.setStyleSheet(get_status_color(False))
        self.refresh_btn = QPushButton("⟳")
        self.refresh_btn.setFixedWidth(30)
        self.refresh_btn.clicked.connect(self.refresh_ports)

        top_hbox = QHBoxLayout()
        top_hbox.addWidget(QLabel("Puerto:"))
        top_hbox.addWidget(self.port_combo)
        top_hbox.addWidget(self.refresh_btn)
        top_hbox.addWidget(self.connect_btn)
        top_hbox.addWidget(self.status_label)
        top_hbox.addStretch()

        # --------- Panel de video ---------
        self.video_label = QLabel()
        self.video_label.setFixedSize(320, 240)
        video_box = QGroupBox("Video")
        vbox_video = QVBoxLayout()
        vbox_video.addWidget(self.video_label)
        self.save_img_btn = QPushButton("Guardar Imagen")
        self.save_img_btn.clicked.connect(self.save_image)
        vbox_video.addWidget(self.save_img_btn)
        video_box.setLayout(vbox_video)

        # --------- Panel de cubo 3D ---------
        self.fig = plt.figure(figsize=(3,3))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvas(self.fig)
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.init_cube()
        self.pitch_label = QLabel("Pitch: 0.0°")
        self.roll_label = QLabel("Roll: 0.0°")
        self.yaw_label = QLabel("Yaw: 0.0°")
        vbox_cubo = QVBoxLayout()
        vbox_cubo.addWidget(self.canvas)
        vbox_cubo.addWidget(self.pitch_label)
        vbox_cubo.addWidget(self.roll_label)
        vbox_cubo.addWidget(self.yaw_label)
        cubo_box = QGroupBox("Actitud (IMU)")
        cubo_box.setLayout(vbox_cubo)

        # --------- Panel de telemetría ---------
        self.bat_bar = QProgressBar()
        self.bat_bar.setMaximum(100)
        self.bat_bar.setValue(100)
        self.temp_label = QLabel("Temp: -- °C")
        self.alt_label = QLabel("Altitud: -- m")
        self.state_label = QLabel("Estado: --")
        grid_tele = QGridLayout()
        grid_tele.addWidget(QLabel("Batería:"), 0, 0)
        grid_tele.addWidget(self.bat_bar, 0, 1)
        grid_tele.addWidget(self.temp_label, 1, 0)
        grid_tele.addWidget(self.alt_label, 1, 1)
        grid_tele.addWidget(self.state_label, 2, 0, 1, 2)
        tele_box = QGroupBox("Telemetría")
        tele_box.setLayout(grid_tele)

        # --------- Botones de control ---------
        self.pause_btn = QPushButton("Pausar")
        self.pause_btn.setCheckable(True)
        self.save_log_btn = QPushButton("Guardar Log")
        self.save_log_btn.clicked.connect(self.save_log)
        self.calib_btn = QPushButton("Calibrar IMU")
        self.reset_btn = QPushButton("Reset")
        hbox_ctrl = QHBoxLayout()
        hbox_ctrl.addWidget(self.pause_btn)
        hbox_ctrl.addWidget(self.save_log_btn)
        hbox_ctrl.addWidget(self.calib_btn)
        hbox_ctrl.addWidget(self.reset_btn)
        hbox_ctrl.addStretch()

        # --------- Layout principal ---------
        grid = QGridLayout()
        grid.addLayout(top_hbox, 0, 0, 1, 2)
        grid.addWidget(video_box, 1, 0)
        grid.addWidget(cubo_box, 1, 1)
        grid.addWidget(tele_box, 2, 0, 1, 2)
        grid.addLayout(hbox_ctrl, 3, 0, 1, 2)
        self.setLayout(grid)

        # --------- Timer de actualización ---------
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(30)

        # --------- Estado ---------
        self.last_time = time.time()
        self.alpha = 0.98
        self.pitch = 0.0
        self.roll = 0.0
        self.last_img = None
        self.log_lines = []

    def refresh_ports(self):
        self.port_combo.clear()
        self.port_combo.addItems(list_serial_ports())

    def toggle_connection(self):
        if self.connected:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.port_combo.currentText()
        try:
            self.serial = serial.Serial(port, 115200, timeout=2)
            self.connected = True
            self.status_label.setText("Conectado")
            self.status_label.setStyleSheet(get_status_color(True))
            self.ser_port = port
        except Exception as e:
            self.status_label.setText("Error")
            self.status_label.setStyleSheet(get_status_color(False))
            print("Error al conectar:", e)

    def disconnect_serial(self):
        if self.serial:
            self.serial.close()
        self.connected = False
        self.status_label.setText("Desconectado")
        self.status_label.setStyleSheet(get_status_color(False))
        self.ser_port = None

    def save_image(self):
        if self.last_img is not None:
            filename, _ = QFileDialog.getSaveFileName(self, "Guardar Imagen", "", "JPEG (*.jpg *.jpeg)")
            if filename:
                cv2.imwrite(filename, cv2.cvtColor(self.last_img, cv2.COLOR_RGB2BGR))

    def save_log(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Guardar Log", "", "CSV (*.csv)")
        if filename:
            with open(filename, "w") as f:
                for line in self.log_lines:
                    f.write(line + "\n")

    def init_cube(self):
        self.ax.cla()
        r = [-0.5, 0.5]
        X, Y = np.meshgrid(r, r)
        self.ax.plot_surface(X, Y, np.full_like(X, 0.5), alpha=0.5, color='cyan')
        self.ax.plot_surface(X, Y, np.full_like(X, -0.5), alpha=0.5, color='cyan')
        self.ax.plot_surface(X, np.full_like(X, 0.5), Y, alpha=0.5, color='cyan')
        self.ax.plot_surface(X, np.full_like(X, -0.5), Y, alpha=0.5, color='cyan')
        self.ax.plot_surface(np.full_like(X, 0.5), X, Y, alpha=0.5, color='cyan')
        self.ax.plot_surface(np.full_like(X, -0.5), X, Y, alpha=0.5, color='cyan')
        self.ax.set_xlim([-1,1])
        self.ax.set_ylim([-1,1])
        self.ax.set_zlim([-1,1])
        self.ax.set_box_aspect([1,1,1])
        self.ax.axis('off')
        self.ax.view_init(elev=self.pitch, azim=self.yaw)
        self.canvas.draw()

    def update_cube(self, pitch, roll, yaw=0):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self.init_cube()
        self.pitch_label.setText(f"Pitch: {self.pitch:.1f}°")
        self.roll_label.setText(f"Roll: {self.roll:.1f}°")
        self.yaw_label.setText(f"Yaw: {self.yaw:.1f}°")

    def update_data(self):
        if not self.connected or self.pause_btn.isChecked():
            return
        try:
            # Lee línea de datos del MPU6050
            line = self.serial.readline().decode(errors='ignore').strip()
            if not line.startswith('ACC:'):
                return
            self.log_lines.append(line)
            acc = line.split('ACC:')[1].split(';')[0]
            gyro = line.split('GYRO:')[1].split(';')[0]
            ax_val, ay_val, az_val = [float(x) for x in acc.split(',')]
            gx_val, gy_val, gz_val = [float(x) for x in gyro.split(',')]
            # Convierte a unidades físicas
            ax_val /= 16384.0
            ay_val /= 16384.0
            az_val /= 16384.0
            gx_val /= 131.0
            gy_val /= 131.0
            gz_val /= 131.0
            now = time.time()
            dt = now - self.last_time
            self.last_time = now
            # Pitch y roll del acelerómetro
            pitch_acc = math.atan2(-ax_val, math.sqrt(ay_val**2 + az_val**2)) * 180 / math.pi
            roll_acc = math.atan2(ay_val, az_val) * 180 / math.pi
            # Filtro complementario
            self.pitch = self.alpha * (self.pitch + gy_val * dt) + (1 - self.alpha) * pitch_acc
            self.roll = self.alpha * (self.roll + gx_val * dt) + (1 - self.alpha) * roll_acc
            self.yaw += gz_val * dt
            self.update_cube(self.pitch, self.roll, self.yaw)
            # Lee imagen
            while self.serial.read(1) != b'\xAA':
                pass
            size_bytes = self.serial.read(2)
            img_size = (size_bytes[0] << 8) | size_bytes[1]
            img_data = b''
            while len(img_data) < img_size:
                img_data += self.serial.read(img_size - len(img_data))
            img_array = np.frombuffer(img_data, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if img is not None:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                h, w, ch = img.shape
                bytes_per_line = ch * w
                qt_img = QImage(img.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.video_label.setPixmap(QPixmap.fromImage(qt_img))
                self.last_img = img
            # Simula telemetría (puedes actualizar con tus datos reales)
            self.bat_bar.setValue(90)
            self.temp_label.setText("Temp: 25.0 °C")
            self.alt_label.setText("Altitud: 123 m")
            self.state_label.setText("Estado: En vuelo")
        except Exception as e:
            print("Error en update_data:", e)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    window = MainWindow()
    window.show()
    app.exec_()