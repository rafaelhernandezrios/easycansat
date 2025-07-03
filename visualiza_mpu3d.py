import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import time

ser = serial.Serial('COM11', 115200, timeout=2)

# Filtro complementario
alpha = 0.98
pitch = 0.0
roll = 0.0
last_time = time.time()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Función para dibujar el cubo
r = [-0.5, 0.5]
X, Y = np.meshgrid(r, r)
def draw_cube(ax, pitch, roll):
    ax.cla()
    # Caras Z
    ax.plot_surface(X, Y, np.full_like(X, 0.5), alpha=0.5, color='cyan')
    ax.plot_surface(X, Y, np.full_like(X, -0.5), alpha=0.5, color='cyan')
    # Caras Y
    ax.plot_surface(X, np.full_like(X, 0.5), Y, alpha=0.5, color='cyan')
    ax.plot_surface(X, np.full_like(X, -0.5), Y, alpha=0.5, color='cyan')
    # Caras X
    ax.plot_surface(np.full_like(X, 0.5), X, Y, alpha=0.5, color='cyan')
    ax.plot_surface(np.full_like(X, -0.5), X, Y, alpha=0.5, color='cyan')
    ax.set_xlim([-1,1])
    ax.set_ylim([-1,1])
    ax.set_zlim([-1,1])
    ax.set_box_aspect([1,1,1])
    ax.axis('off')
    # Aplica rotación
    ax.view_init(elev=pitch, azim=roll)
    plt.draw()
    plt.pause(0.001)

while True:
    line = ser.readline().decode(errors='ignore').strip()
    if line.startswith('ACC:'):
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
            draw_cube(ax, pitch, roll)
        except Exception as e:
            print('Error parsing line:', line, e) 