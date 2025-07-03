import sys
import serial
import numpy as np
import cv2
import math
import time
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QHBoxLayout, QPushButton
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# Configura el puerto serial
ser = serial.Serial('COM11', 115200, timeout=2)

# Filtro complementario
alpha = 0.98
pitch = 0.0
roll = 0.0
last_time = time.time()

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Video + Cubo 3D MPU6050")
        self.video_label = QLabel()
        self.video_label.setFixedSize(320, 240)
        self.mpu_label = QLabel("MPU6050: ---")
        self.mpu_label.setFixedHeight(20)

        # Matplotlib 3D figure
        self.fig = plt.figure(figsize=(3,3))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvas(self.fig)
        self.pitch = 0.0
        self.roll = 0.0
        self.last_time = time.time()
        self.init_cube()

        # Layouts
        hbox = QHBoxLayout()
        hbox.addWidget(self.video_label)
        hbox.addWidget(self.canvas)

        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        vbox.addWidget(self.mpu_label)
        self.setLayout(vbox)

        # Timer para actualizar
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(30)  # ~33 fps

    def init_cube(self):
        self.ax.cla()
        r = [-0.5, 0.5]
        X, Y = np.meshgrid(r, r)
        # Caras Z = constante
        self.ax.plot_surface(X, Y, np.full_like(X, 0.5), alpha=0.5, color='cyan')
        self.ax.plot_surface(X, Y, np.full_like(X, -0.5), alpha=0.5, color='cyan')
        # Caras Y = constante
        self.ax.plot_surface(X, np.full_like(X, 0.5), Y, alpha=0.5, color='cyan')
        self.ax.plot_surface(X, np.full_like(X, -0.5), Y, alpha=0.5, color='cyan')
        # Caras X = constante
        self.ax.plot_surface(np.full_like(X, 0.5), X, Y, alpha=0.5, color='cyan')
        self.ax.plot_surface(np.full_like(X, -0.5), X, Y, alpha=0.5, color='cyan')
        self.ax.set_xlim([-1,1])
        self.ax.set_ylim([-1,1])
        self.ax.set_zlim([-1,1])
        self.ax.set_box_aspect([1,1,1])
        self.ax.axis('off')
        self.ax.view_init(elev=self.pitch, azim=self.roll)
        self.canvas.draw()

    def update_cube(self, pitch, roll):
        self.pitch = pitch
        self.roll = roll
        self.init_cube()

    def update_data(self):
        global pitch, roll, last_time
        # Lee línea de datos del MPU6050
        line = ser.readline().decode(errors='ignore').strip()
        if not line.startswith('ACC:'):
            return
        try:
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
            self.pitch = alpha * (self.pitch + gy_val * dt) + (1 - alpha) * pitch_acc
            self.roll = alpha * (self.roll + gx_val * dt) + (1 - alpha) * roll_acc
            self.mpu_label.setText(f"MPU6050: pitch={self.pitch:.1f} roll={self.roll:.1f}")
        except Exception as e:
            self.mpu_label.setText(f"Error parsing: {e}")
            return
        # Lee imagen
        # Sincroniza con el byte de inicio
        while True:
            byte = ser.read(1)
            if byte == b'':
                print("Timeout esperando imagen")
                return
            if byte == b'\xAA':
                break

        size_bytes = ser.read(2)
        if len(size_bytes) < 2:
            print("No se pudo leer el tamaño de la imagen")
            return
        img_size = (size_bytes[0] << 8) | size_bytes[1]
        img_data = b''
        while len(img_data) < img_size:
            chunk = ser.read(img_size - len(img_data))
            if not chunk:
                print("Timeout leyendo imagen")
                return
            img_data += chunk

        # Intenta decodificar la imagen
        img_array = np.frombuffer(img_data, dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        if img is not None:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            h, w, ch = img.shape
            bytes_per_line = ch * w
            qt_img = QImage(img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qt_img))
        else:
            print("Imagen corrupta, descartando")
        self.update_cube(self.pitch, self.roll)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_()) 