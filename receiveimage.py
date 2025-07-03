import sys
import serial
import cv2
import numpy as np
import threading
import time
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QHBoxLayout, QPushButton
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from collections import deque
import math

# Configura el puerto serial
ser = serial.Serial('COM11', 115200, timeout=5)  # Cambia 'COM3' por tu puerto

def parse_mpu_line(line):
    # Espera formato: ACC:x,y,z;GYRO:x,y,z;
    try:
        acc = line.split('ACC:')[1].split(';')[0]
        gyro = line.split('GYRO:')[1].split(';')[0]
        ax, ay, az = [float(x) for x in acc.split(',')]
        gx, gy, gz = [float(x) for x in gyro.split(',')]
        return ax, ay, az, gx, gy, gz
    except Exception:
        return 0, 0, 0, 0, 0, 0

class DataReceiver(threading.Thread):
    def __init__(self):
        super().__init__()
        self.lock = threading.Lock()
        self.latest_mpu = None
        self.latest_img = None
        self.running = True
        # Calibración
        self.calibrating = True
        self.calib_samples = []
        self.calib_time = 2.0  # segundos de calibración
        self.calib_min_samples = 30  # mínimo de muestras para calibrar
        self.calib_start = None
        self.offsets = None
        self.noise = None

    def run(self):
        while self.running:
            try:
                # Lee línea del MPU6050
                mpu_line = ser.readline().decode(errors='ignore').strip()
                if mpu_line.startswith("ACC:"):
                    if self.calibrating:
                        if self.calib_start is None:
                            self.calib_start = time.time()
                        ax, ay, az, gx, gy, gz = parse_mpu_line(mpu_line)
                        self.calib_samples.append([ax, ay, az, gx, gy, gz])
                        # Lee y descarta la imagen para mantener el flujo
                        while ser.read(1) != b'\xAA':
                            pass
                        size_bytes = ser.read(2)
                        img_size = (size_bytes[0] << 8) | size_bytes[1]
                        img_data = b''
                        while len(img_data) < img_size:
                            chunk = ser.read(img_size - len(img_data))
                            if not chunk:
                                break
                            img_data += chunk
                        # Termina calibración si pasa el tiempo o si hay suficientes muestras
                        if (time.time() - self.calib_start > self.calib_time) or (len(self.calib_samples) >= self.calib_min_samples):
                            arr = np.array(self.calib_samples)
                            self.offsets = arr.mean(axis=0)
                            self.noise = arr.std(axis=0)
                            self.calibrating = False
                            print("Calibración terminada. Offsets:", self.offsets, "Ruido:", self.noise)
                        continue
                    with self.lock:
                        self.latest_mpu = mpu_line
                    # Espera el byte de inicio de imagen
                    while ser.read(1) != b'\xAA':
                        pass
                    size_bytes = ser.read(2)
                    img_size = (size_bytes[0] << 8) | size_bytes[1]
                    img_data = b''
                    while len(img_data) < img_size:
                        chunk = ser.read(img_size - len(img_data))
                        if not chunk:
                            break
                        img_data += chunk
                    with self.lock:
                        self.latest_img = img_data
            except Exception as e:
                print("Error en hilo de recepción:", e)
                time.sleep(0.1)

    def get_latest(self):
        with self.lock:
            mpu = self.latest_mpu
            img = self.latest_img
            self.latest_img = None  # Solo muestra cada imagen una vez
        return mpu, img

    def get_calibration(self):
        return self.calibrating, self.offsets, self.noise

    def stop(self):
        self.running = False

class MainWindow(QWidget):
    def __init__(self, data_receiver):
        super().__init__()
        self.setWindowTitle("ESP32S3 Video + MPU6050 Visualizer")
        self.video_label = QLabel()
        self.mpu_label = QLabel("MPU6050: ---")
        self.mpu_label.setFixedHeight(20)

        # Matplotlib 3D figure
        self.fig = plt.figure(figsize=(3,3))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvas(self.fig)
        self.init_cube()

        # Layouts
        hbox = QHBoxLayout()
        hbox.addWidget(self.video_label)
        hbox.addWidget(self.canvas)

        vbox = QVBoxLayout()
        vbox.addLayout(hbox)
        vbox.addWidget(self.mpu_label)

        # Calibración multiorientación
        self.calib_steps = [
            ("Z+", "Pon el sensor con la cara Z+ hacia arriba y presiona 'Tomar muestra'"),
            ("Z-", "Pon el sensor con la cara Z- hacia arriba y presiona 'Tomar muestra'"),
            ("X+", "Pon el sensor con la cara X+ hacia arriba y presiona 'Tomar muestra'"),
            ("X-", "Pon el sensor con la cara X- hacia arriba y presiona 'Tomar muestra'"),
            ("Y+", "Pon el sensor con la cara Y+ hacia arriba y presiona 'Tomar muestra'"),
            ("Y-", "Pon el sensor con la cara Y- hacia arriba y presiona 'Tomar muestra'"),
        ]
        self.calib_data = []
        self.current_step = 0
        self.calibrating = True

        self.calib_button = QPushButton("Tomar muestra")
        self.calib_button.clicked.connect(self.take_calib_sample)
        vbox.addWidget(self.calib_button)

        self.update_calib_instruction()

        self.setLayout(vbox)

        self.data_receiver = data_receiver

        # Filtros
        self.N = 30
        self.alpha = 0.05
        self.ax_hist = deque([0]*self.N, maxlen=self.N)
        self.ay_hist = deque([0]*self.N, maxlen=self.N)
        self.az_hist = deque([0]*self.N, maxlen=self.N)
        self.gx_hist = deque([0]*self.N, maxlen=self.N)
        self.gy_hist = deque([0]*self.N, maxlen=self.N)
        self.gz_hist = deque([0]*self.N, maxlen=self.N)
        self.ax_filt = 0
        self.ay_filt = 0
        self.az_filt = 0
        self.gx_filt = 0
        self.gy_filt = 0
        self.gz_filt = 0
        self.deadzone_factor = 2  # Multiplicador del ruido para zona muerta
        self.offsets = np.zeros(6)
        self.noise = np.ones(6)

        # Timer para actualizar
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(30)  # ~33 fps

        # Nueva variable para el giroscopio
        self.pitch = 0.0
        self.roll = 0.0
        self.last_time = time.time()
        self.alpha_comp = 0.98  # Peso del giroscopio

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

    def update_cube(self, ax, ay, az):
        self.init_cube()
        # Para una visualización simple, usa los valores como ángulos de Euler
        self.ax.view_init(elev=ay, azim=ax)
        self.canvas.draw()

    def apply_deadzone(self, val, noise):
        return val if abs(val) > self.deadzone_factor * noise else 0

    def update_calib_instruction(self):
        if self.current_step < len(self.calib_steps):
            self.mpu_label.setText(self.calib_steps[self.current_step][1])
        else:
            self.mpu_label.setText("¡Calibración completa!")
            self.calib_button.setEnabled(False)
            self.finish_calibration()

    def take_calib_sample(self):
        samples = []
        last_mpu_line = None
        timeout = time.time() + 5  # 5 segundos máximo para tomar muestras
        while len(samples) < 30 and time.time() < timeout:
            mpu_line, _ = self.data_receiver.get_latest()
            if mpu_line and mpu_line != last_mpu_line:
                ax, ay, az, gx, gy, gz = parse_mpu_line(mpu_line)
                samples.append([ax, ay, az])
                last_mpu_line = mpu_line
            else:
                time.sleep(0.01)
            QApplication.processEvents()
        if len(samples) == 0:
            self.mpu_label.setText("No se recibieron muestras, intenta de nuevo")
            return
        avg = np.mean(samples, axis=0)
        if avg.shape == ():  # Si es escalar, ignóralo
            self.mpu_label.setText("Error: promedio vacío, intenta de nuevo")
            return
        self.calib_data.append(avg)
        self.current_step += 1
        self.update_calib_instruction()

    def finish_calibration(self):
        if len(self.calib_data) < 6:
            self.mpu_label.setText("Error: faltan muestras de calibración")
            return
        # Calcula offset y escala para cada eje
        # Z+: 0, Z-:1, X+:2, X-:3, Y+:4, Y-:5
        z_offset = (self.calib_data[0][2] + self.calib_data[1][2]) / 2
        x_offset = (self.calib_data[2][0] + self.calib_data[3][0]) / 2
        y_offset = (self.calib_data[4][1] + self.calib_data[5][1]) / 2
        z_scale = (self.calib_data[0][2] - self.calib_data[1][2]) / 2
        x_scale = (self.calib_data[2][0] - self.calib_data[3][0]) / 2
        y_scale = (self.calib_data[4][1] - self.calib_data[5][1]) / 2
        self.offsets = np.array([x_offset, y_offset, z_offset])
        self.scales = np.array([x_scale, y_scale, z_scale])
        self.calibrating = False
        print(f'Offsets: {self.offsets}, Scales: {self.scales}')

    def update_data(self):
        if self.calibrating:
            return  # No actualices visualización hasta terminar calibración
        mpu_line, img_data = self.data_receiver.get_latest()
        if mpu_line:
            self.mpu_label.setText("MPU6050: " + mpu_line)
            ax, ay, az, gx, gy, gz = parse_mpu_line(mpu_line)
            # Aplica offset y escala
            acc = np.array([ax, ay, az])
            acc = (acc - self.offsets) / self.scales
            # Zona muerta dinámica
            ax = self.apply_deadzone(acc[0], self.noise[0])
            ay = self.apply_deadzone(acc[1], self.noise[1])
            az = self.apply_deadzone(acc[2], self.noise[2])
            gx = self.apply_deadzone(gx, self.noise[3])
            gy = self.apply_deadzone(gy, self.noise[4])
            gz = self.apply_deadzone(gz, self.noise[5])
            # Filtro de media móvil
            self.ax_hist.append(ax)
            self.ay_hist.append(ay)
            self.az_hist.append(az)
            self.gx_hist.append(gx)
            self.gy_hist.append(gy)
            self.gz_hist.append(gz)
            ax_avg = sum(self.ax_hist) / self.N
            ay_avg = sum(self.ay_hist) / self.N
            az_avg = sum(self.az_hist) / self.N
            gx_avg = sum(self.gx_hist) / self.N
            gy_avg = sum(self.gy_hist) / self.N
            gz_avg = sum(self.gz_hist) / self.N
            # Filtro exponencial
            self.ax_filt = self.alpha * ax_avg + (1 - self.alpha) * self.ax_filt
            self.ay_filt = self.alpha * ay_avg + (1 - self.alpha) * self.ay_filt
            self.az_filt = self.alpha * az_avg + (1 - self.alpha) * self.az_filt
            self.gx_filt = self.alpha * gx_avg + (1 - self.alpha) * self.gx_filt
            self.gy_filt = self.alpha * gy_avg + (1 - self.alpha) * self.gy_filt
            self.gz_filt = self.alpha * gz_avg + (1 - self.alpha) * self.gz_filt
            # Visualización
            self.update_cube(self.ax_filt, self.ay_filt, self.az_filt)
        if img_data:
            img_array = np.frombuffer(img_data, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if img is not None:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                h, w, ch = img.shape
                bytes_per_line = ch * w
                qt_img = QImage(img.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.video_label.setPixmap(QPixmap.fromImage(qt_img))

            # Calcula dt
            now = time.time()
            dt = now - self.last_time if hasattr(self, 'last_time') else 0.03
            self.last_time = now

            # Acelerómetro: calcula pitch y roll
            pitch_acc = math.atan2(-self.ax_filt, math.sqrt(self.ay_filt**2 + self.az_filt**2)) * 180 / math.pi
            roll_acc = math.atan2(self.ay_filt, self.az_filt) * 180 / math.pi

            # Giroscopio: asume que gx, gy están en grados/segundo
            self.pitch = self.alpha_comp * (self.pitch + self.gy_filt * dt) + (1 - self.alpha_comp) * pitch_acc
            self.roll = self.alpha_comp * (self.roll + self.gx_filt * dt) + (1 - self.alpha_comp) * roll_acc

            # Visualiza el cubo usando self.pitch y self.roll
            self.update_cube(self.roll, self.pitch, 0)

if __name__ == "__main__":
    data_receiver = DataReceiver()
    data_receiver.start()
    app = QApplication(sys.argv)
    window = MainWindow(data_receiver)
    window.show()
    app.exec_()
    data_receiver.stop()
    data_receiver.join()