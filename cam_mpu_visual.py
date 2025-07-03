import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import time
import cv2

ser = serial.Serial('COM11', 115200, timeout=2)

# Filtro complementario
alpha = 0.98
pitch = 0.0
roll = 0.0
last_time = time.time()

# Matplotlib 3D
fig = plt.figure(figsize=(8,4))
ax1 = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122)
plt.ion()

# Función para dibujar el cubo
r = [-0.5, 0.5]
X, Y = np.meshgrid(r, r)
def draw_cube(ax, pitch, roll):
    ax.cla()
    ax.plot_surface(X, Y, np.full_like(X, 0.5), alpha=0.5, color='cyan')
    ax.plot_surface(X, Y, np.full_like(X, -0.5), alpha=0.5, color='cyan')
    ax.plot_surface(X, np.full_like(X, 0.5), Y, alpha=0.5, color='cyan')
    ax.plot_surface(X, np.full_like(X, -0.5), Y, alpha=0.5, color='cyan')
    ax.plot_surface(np.full_like(X, 0.5), X, Y, alpha=0.5, color='cyan')
    ax.plot_surface(np.full_like(X, -0.5), X, Y, alpha=0.5, color='cyan')
    ax.set_xlim([-1,1])
    ax.set_ylim([-1,1])
    ax.set_zlim([-1,1])
    ax.set_box_aspect([1,1,1])
    ax.axis('off')
    ax.view_init(elev=pitch, azim=roll)

def show_image(ax, img):
    ax.cla()
    ax.axis('off')
    ax.imshow(img)

while True:
    # Lee línea de datos del MPU6050
    line = ser.readline().decode(errors='ignore').strip()
    if not line.startswith('ACC:'):
        continue
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
        dt = now - last_time
        last_time = now
        # Pitch y roll del acelerómetro
        pitch_acc = math.atan2(-ax_val, math.sqrt(ay_val**2 + az_val**2)) * 180 / math.pi
        roll_acc = math.atan2(ay_val, az_val) * 180 / math.pi
        # Filtro complementario
        pitch = alpha * (pitch + gy_val * dt) + (1 - alpha) * pitch_acc
        roll = alpha * (roll + gx_val * dt) + (1 - alpha) * roll_acc
    except Exception as e:
        print('Error parsing line:', line, e)
        continue
    # Lee imagen
    while ser.read(1) != b'\xAA':
        pass
    size_bytes = ser.read(2)
    img_size = (size_bytes[0] << 8) | size_bytes[1]
    img_data = b''
    while len(img_data) < img_size:
        img_data += ser.read(img_size - len(img_data))
    img_array = np.frombuffer(img_data, dtype=np.uint8)
    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    if img is not None:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        show_image(ax2, img)
    draw_cube(ax1, pitch, roll)
    plt.pause(0.001) 