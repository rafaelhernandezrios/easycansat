import sys
import serial
import serial.tools.list_ports
import numpy as np
import cv2
import math
import time
from PyQt5.QtWidgets import (
    QApplication, QLabel, QVBoxLayout, QWidget, QHBoxLayout, QPushButton,
    QComboBox, QGroupBox, QGridLayout, QLineEdit, QTabWidget, QProgressBar, QFileDialog, QSpacerItem, QSizePolicy
)
from PyQt5.QtGui import QImage, QPixmap, QColor  

from PyQt5.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import Figure
import qdarkstyle
import os
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl
import folium
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

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
        # En __init__(), junto a los demás:
        self.mini_time = []
        self.mini_accel = []
        self.mini_max_pts = 50

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

        # --------- Crear pestañas ---------
        self.tab_widget = QTabWidget()
        
        # Pestaña 1: Dashboard principal
        self.dashboard_tab = QWidget()
        self.setup_dashboard_tab()
        
        # Pestaña 2: Gráficas IMU en tiempo real
        self.graphs_tab = QWidget()
        self.setup_graphs_tab()
        
        self.tab_widget.addTab(self.dashboard_tab, "Dashboard")
        self.tab_widget.addTab(self.graphs_tab, "Gráficas IMU")
        
        # Conectar el cambio de pestaña para actualizar gráficas
        self.tab_widget.currentChanged.connect(self.on_tab_changed)

        # --------- Layout principal ---------
        main_vbox = QVBoxLayout()
        main_vbox.addLayout(top_hbox)
        main_vbox.addWidget(self.tab_widget)
        self.setLayout(main_vbox)

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
        
        # --------- Datos para gráficas ---------
        self.time_data = []
        self.accel_x_data = []
        self.accel_y_data = []
        self.accel_z_data = []
        self.gyro_x_data = []
        self.gyro_y_data = []
        self.gyro_z_data = []
        self.max_points = 100  # Número máximo de puntos en las gráficas
        
        self.setStyleSheet("""
            QWidget {
                background-color: #18304b;
                color: #7fd6ff;
                font-family: Arial, Helvetica, sans-serif;
            }
            QGroupBox {
                border: 2px solid #2a4d6c;
                border-radius: 15px;
                margin-top: 10px;
                font-weight: bold;
                font-size: 18px;
                color: #7fd6ff;
                background-color: #1b3957;
            }
            QLabel {
                color: #7fd6ff;
                font-size: 16px;
            }
            QPushButton {
                background-color: #2a4d6c;
                color: #7fd6ff;
                border-radius: 10px;
                padding: 6px 12px;
                font-size: 15px;
            }
            QProgressBar {
                background-color: #1b3957;
                color: #7fd6ff;
                border: 1px solid #2a4d6c;
                border-radius: 10px;
                text-align: center;
            }
            QTabWidget::pane {
                border: 2px solid #2a4d6c;
                border-radius: 15px;
                background-color: #1b3957;
            }
            QTabBar::tab {
                background-color: #2a4d6c;
                color: #7fd6ff;
                padding: 8px 16px;
                margin-right: 2px;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
            }
            QTabBar::tab:selected {
                background-color: #1b3957;
                border-bottom: 2px solid #7fd6ff;
            }
        """)

    def setup_dashboard_tab(self):
        """Configura la pestaña del dashboard principal"""
        # --------- Panel de video ---------
        self.video_label = QLabel()
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_box = QGroupBox("Video")
        vbox_video = QVBoxLayout()
        vbox_video.addWidget(self.video_label)
        self.save_img_btn = QPushButton("Guardar Imagen")
        self.save_img_btn.clicked.connect(self.save_image)
        vbox_video.addWidget(self.save_img_btn)
        video_box.setLayout(vbox_video)

        # --------- Panel de cubo 3D ---------
        self.fig = plt.figure(figsize=(5,5))
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
        cubo_box.setMinimumSize(350, 350)

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

        # --------- Panel de sensores (nuevo estilo) ---------
        sensors_box = QGroupBox()
        sensors_layout = QVBoxLayout()
        sensors_title = QLabel("SENSORS")
        sensors_title.setStyleSheet("font-size: 22px; color: #7fd6ff; font-weight: bold; margin-bottom: 10px;")
        sensors_layout.addWidget(sensors_title)

        # --------- Panel de gráficas IMU en tiempo real (mini) ---------
        self.mini_fig = Figure(figsize=(4,3))
        self.mini_ax = self.mini_fig.add_subplot(111)
        self.mini_ax.set_facecolor('#1b3957')
        self.mini_ax.set_title('IMU (g)', color='#7fd6ff', fontsize=12)
        self.mini_ax.tick_params(colors='#7fd6ff')
        self.mini_canvas = FigureCanvas(self.mini_fig)
        mini_box = QGroupBox("Gráficas IMU")
        vbox_mini = QVBoxLayout()
        vbox_mini.addWidget(self.mini_canvas)
        mini_box.setLayout(vbox_mini)
        mini_box.setMinimumSize(500,
                                400)
        mini_box.setStyleSheet("padding: 12px; border-radius: 18px; background-color: #1b3957;")

        # ALTITUDE
        altitude_row = QHBoxLayout()
        altitude_label = QLabel("ALTITUDE")
        altitude_label.setStyleSheet("font-size: 16px; color: #7fd6ff;")
        self.altitude_value = QLabel("316")
        self.altitude_value.setStyleSheet("font-size: 28px; color: #7fd6ff; font-weight: bold;")
        altitude_row.addWidget(altitude_label)
        altitude_row.addStretch()
        altitude_row.addWidget(self.altitude_value)
        sensors_layout.addLayout(altitude_row)

        # PRESSURE
        pressure_row = QHBoxLayout()
        pressure_label = QLabel("PRESSURE")
        pressure_label.setStyleSheet("font-size: 16px; color: #7fd6ff;")
        self.pressure_value = QLabel("987.4")
        self.pressure_value.setStyleSheet("font-size: 28px; color: #7fd6ff; font-weight: bold;")
        pressure_row.addWidget(pressure_label)
        pressure_row.addStretch()
        pressure_row.addWidget(self.pressure_value)
        sensors_layout.addLayout(pressure_row)

        # ACCELERATION
        accel_row = QHBoxLayout()
        accel_label = QLabel("ACCELERATION")
        accel_label.setStyleSheet("font-size: 16px; color: #7fd6ff;")
        self.accel_value = QLabel("0.5")
        self.accel_value.setStyleSheet("font-size: 28px; color: #7fd6ff; font-weight: bold;")
        accel_row.addWidget(accel_label)
        accel_row.addStretch()
        accel_row.addWidget(self.accel_value)
        sensors_layout.addLayout(accel_row)

        sensors_box.setLayout(sensors_layout)

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

        # --------- Panel de mapa (folium + QWebEngineView) ---------
        # Ubicación predeterminada: Ciudad de México
        lat, lon = 19.4326, -99.1332
        m = folium.Map(location=[lat, lon], zoom_start=15)
        map_path = os.path.abspath('map.html')
        m.save(map_path)
        self.map_view = QWebEngineView()
        self.map_view.setMinimumSize(400, 300)
        self.map_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.map_view.load(QUrl.fromLocalFile(map_path))
        map_box = QGroupBox("Mapa")
        vbox_map = QVBoxLayout()
        vbox_map.addWidget(self.map_view)
        map_box.setLayout(vbox_map)
        map_box.setMinimumSize(400, 300)
        map_box.setStyleSheet("padding: 18px; border-radius: 18px; background-color: #1b3957;")

        # --------- Layout superior: SENSORS | ATTITUDE | MAPA | VIDEO ---------
        top_panels = QHBoxLayout()
        top_panels.addSpacing(10)
        top_panels.addWidget(sensors_box, stretch=1)
        top_panels.addSpacing(10)
        top_panels.addWidget(cubo_box, stretch=2)
        top_panels.addSpacing(10)
        top_panels.addWidget(map_box, stretch=2)
        top_panels.addSpacing(10)
        top_panels.addWidget(video_box, stretch=2)
        top_panels.addSpacing(10)
        top_panels.addWidget(mini_box, stretch=1)

        # Layout central: TELEMETRY (sin márgenes laterales)
        center_panels = QHBoxLayout()
        center_panels.addWidget(tele_box, stretch=10)

        # Layout inferior: Botones de control
        bottom_panels = QHBoxLayout()
        bottom_panels.addSpacing(10)
        bottom_panels.addLayout(hbox_ctrl)
        bottom_panels.addSpacing(10)

        # Layout vertical principal para dashboard
        dashboard_vbox = QVBoxLayout()
        dashboard_vbox.addSpacing(10)
        dashboard_vbox.addLayout(top_panels, stretch=4)
        dashboard_vbox.addSpacing(10)
        dashboard_vbox.addLayout(center_panels, stretch=1)
        dashboard_vbox.addSpacing(10)
        dashboard_vbox.addLayout(bottom_panels)
        dashboard_vbox.addStretch()
        self.dashboard_tab.setLayout(dashboard_vbox)

        # Mejoro la estética de los paneles
        sensors_box.setStyleSheet(sensors_box.styleSheet() + "padding: 18px; border-radius: 18px; background-color: #1b3957;")
        cubo_box.setStyleSheet(cubo_box.styleSheet() + "padding: 18px; border-radius: 18px; background-color: #1b3957;")
        video_box.setStyleSheet(video_box.styleSheet() + "padding: 34px; border-radius: 34px; background-color: #1b3957;")
        tele_box.setStyleSheet(tele_box.styleSheet() + "padding: 12px; border-radius: 18px; background-color: #1b3957;")
        map_box.setStyleSheet(map_box.styleSheet() + "padding: 18px; border-radius: 18px; background-color: #1b3957;")

    def setup_graphs_tab(self):
        """Configura la pestaña de gráficas IMU en tiempo real"""
        # Crear figura con subplots para acelerómetro y giroscopio
        self.graph_fig = Figure(figsize=(12, 8))
        self.graph_fig.patch.set_facecolor('#1b3957')
        
        # Subplot para acelerómetro
        self.accel_ax = self.graph_fig.add_subplot(2, 1, 1)
        self.accel_ax.set_facecolor('#1b3957')
        self.accel_ax.set_title('Acelerómetro (g)', color='#7fd6ff', fontsize=14, fontweight='bold')
        self.accel_ax.set_xlabel('Tiempo (s)', color='#7fd6ff')
        self.accel_ax.set_ylabel('Aceleración (g)', color='#7fd6ff')
        self.accel_ax.tick_params(colors='#7fd6ff')
        self.accel_ax.grid(True, alpha=0.3, color='#2a4d6c')
        
        # Subplot para giroscopio
        self.gyro_ax = self.graph_fig.add_subplot(2, 1, 2)
        self.gyro_ax.set_facecolor('#1b3957')
        self.gyro_ax.set_title('Giroscopio (°/s)', color='#7fd6ff', fontsize=14, fontweight='bold')
        self.gyro_ax.set_xlabel('Tiempo (s)', color='#7fd6ff')
        self.gyro_ax.set_ylabel('Velocidad Angular (°/s)', color='#7fd6ff')
        self.gyro_ax.tick_params(colors='#7fd6ff')
        self.gyro_ax.grid(True, alpha=0.3, color='#2a4d6c')
        
        # Ajustar espaciado entre subplots
        self.graph_fig.tight_layout()
        
        # Canvas para las gráficas
        self.graph_canvas = FigureCanvas(self.graph_fig)
        
        # Toolbar para navegación
        self.graph_toolbar = NavigationToolbar(self.graph_canvas, self.graphs_tab)
        
        # Botones de control para las gráficas
        self.clear_graphs_btn = QPushButton("Limpiar Gráficas")
        self.clear_graphs_btn.clicked.connect(self.clear_graphs)
        self.save_graphs_btn = QPushButton("Guardar Gráficas")
        self.save_graphs_btn.clicked.connect(self.save_graphs)
        
        # Layout para la pestaña de gráficas
        graphs_layout = QVBoxLayout()
        graphs_layout.addWidget(self.graph_toolbar)
        graphs_layout.addWidget(self.graph_canvas)
        
        # Layout para botones
        graphs_buttons_layout = QHBoxLayout()
        graphs_buttons_layout.addWidget(self.clear_graphs_btn)
        graphs_buttons_layout.addWidget(self.save_graphs_btn)
        graphs_buttons_layout.addStretch()
        
        graphs_layout.addLayout(graphs_buttons_layout)
        self.graphs_tab.setLayout(graphs_layout)

    def clear_graphs(self):
        """Limpia todas las gráficas"""
        self.time_data.clear()
        self.accel_x_data.clear()
        self.accel_y_data.clear()
        self.accel_z_data.clear()
        self.gyro_x_data.clear()
        self.gyro_y_data.clear()
        self.gyro_z_data.clear()
        
        self.accel_ax.clear()
        self.gyro_ax.clear()
        
        # Reconfigurar los ejes
        self.accel_ax.set_facecolor('#1b3957')
        self.accel_ax.set_title('Acelerómetro (g)', color='#7fd6ff', fontsize=14, fontweight='bold')
        self.accel_ax.set_xlabel('Tiempo (s)', color='#7fd6ff')
        self.accel_ax.set_ylabel('Aceleración (g)', color='#7fd6ff')
        self.accel_ax.tick_params(colors='#7fd6ff')
        self.accel_ax.grid(True, alpha=0.3, color='#2a4d6c')
        
        self.gyro_ax.set_facecolor('#1b3957')
        self.gyro_ax.set_title('Giroscopio (°/s)', color='#7fd6ff', fontsize=14, fontweight='bold')
        self.gyro_ax.set_xlabel('Tiempo (s)', color='#7fd6ff')
        self.gyro_ax.set_ylabel('Velocidad Angular (°/s)', color='#7fd6ff')
        self.gyro_ax.tick_params(colors='#7fd6ff')
        self.gyro_ax.grid(True, alpha=0.3, color='#2a4d6c')
        
        self.graph_canvas.draw()

    def save_graphs(self):
        """Guarda las gráficas como imagen"""
        filename, _ = QFileDialog.getSaveFileName(self, "Guardar Gráficas", "", "PNG (*.png);;JPEG (*.jpg);;PDF (*.pdf)")
        if filename:
            self.graph_fig.savefig(filename, dpi=300, bbox_inches='tight', facecolor='#1b3957')

    def on_tab_changed(self, index):
        """Se llama cuando el usuario cambia de pestaña"""
        if index == 1 and len(self.time_data) > 0:  # Pestaña de gráficas y hay datos
            # Actualizar las gráficas con todos los datos acumulados
            if len(self.accel_x_data) > 0:
                time_relative = [t - self.time_data[0] for t in self.time_data]
                
                # Actualizar gráfica del acelerómetro
                self.accel_ax.clear()
                self.accel_ax.set_facecolor('#1b3957')
                self.accel_ax.set_title('Acelerómetro (g)', color='#7fd6ff', fontsize=14, fontweight='bold')
                self.accel_ax.set_xlabel('Tiempo (s)', color='#7fd6ff')
                self.accel_ax.set_ylabel('Aceleración (g)', color='#7fd6ff')
                self.accel_ax.tick_params(colors='#7fd6ff')
                self.accel_ax.grid(True, alpha=0.3, color='#2a4d6c')
                
                self.accel_ax.plot(time_relative, self.accel_x_data, color='#ff6b6b', label='X', linewidth=2)
                self.accel_ax.plot(time_relative, self.accel_y_data, color='#4ecdc4', label='Y', linewidth=2)
                self.accel_ax.plot(time_relative, self.accel_z_data, color='#45b7d1', label='Z', linewidth=2)
                self.accel_ax.legend(loc='upper right', facecolor='#1b3957', edgecolor='#2a4d6c')
                
                # Actualizar gráfica del giroscopio
                self.gyro_ax.clear()
                self.gyro_ax.set_facecolor('#1b3957')
                self.gyro_ax.set_title('Giroscopio (°/s)', color='#7fd6ff', fontsize=14, fontweight='bold')
                self.gyro_ax.set_xlabel('Tiempo (s)', color='#7fd6ff')
                self.gyro_ax.set_ylabel('Velocidad Angular (°/s)', color='#7fd6ff')
                self.gyro_ax.tick_params(colors='#7fd6ff')
                self.gyro_ax.grid(True, alpha=0.3, color='#2a4d6c')
                
                self.gyro_ax.plot(time_relative, self.gyro_x_data, color='#ff6b6b', label='X', linewidth=2)
                self.gyro_ax.plot(time_relative, self.gyro_y_data, color='#4ecdc4', label='Y', linewidth=2)
                self.gyro_ax.plot(time_relative, self.gyro_z_data, color='#45b7d1', label='Z', linewidth=2)
                self.gyro_ax.legend(loc='upper right', facecolor='#1b3957', edgecolor='#2a4d6c')
                
                self.graph_fig.tight_layout()
                self.graph_canvas.draw()

    def update_graphs(self, ax_val, ay_val, az_val, gx_val, gy_val, gz_val):
        """Actualiza las gráficas con nuevos datos del IMU"""
        current_time = time.time()
        
        # Agregar nuevos datos
        self.time_data.append(current_time)
        self.accel_x_data.append(ax_val)
        self.accel_y_data.append(ay_val)
        self.accel_z_data.append(az_val)
        self.gyro_x_data.append(gx_val)
        self.gyro_y_data.append(gy_val)
        self.gyro_z_data.append(gz_val)
        
        # Limitar el número de puntos para evitar problemas de memoria
        if len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            self.accel_x_data.pop(0)
            self.accel_y_data.pop(0)
            self.accel_z_data.pop(0)
            self.gyro_x_data.pop(0)
            self.gyro_y_data.pop(0)
            self.gyro_z_data.pop(0)
        
        # Convertir tiempo relativo (empezando desde 0)
        time_relative = [t - self.time_data[0] for t in self.time_data]
        
        # Actualizar gráfica del acelerómetro
        self.accel_ax.clear()
        self.accel_ax.set_facecolor('#1b3957')
        self.accel_ax.set_title('Acelerómetro (g)', color='#7fd6ff', fontsize=14, fontweight='bold')
        self.accel_ax.set_xlabel('Tiempo (s)', color='#7fd6ff')
        self.accel_ax.set_ylabel('Aceleración (g)', color='#7fd6ff')
        self.accel_ax.tick_params(colors='#7fd6ff')
        self.accel_ax.grid(True, alpha=0.3, color='#2a4d6c')
        
        if len(time_relative) > 0:
            self.accel_ax.plot(time_relative, self.accel_x_data, color='#ff6b6b', label='X', linewidth=2)
            self.accel_ax.plot(time_relative, self.accel_y_data, color='#4ecdc4', label='Y', linewidth=2)
            self.accel_ax.plot(time_relative, self.accel_z_data, color='#45b7d1', label='Z', linewidth=2)
            self.accel_ax.legend(loc='upper right', facecolor='#1b3957', edgecolor='#2a4d6c')
        
        # Actualizar gráfica del giroscopio
        self.gyro_ax.clear()
        self.gyro_ax.set_facecolor('#1b3957')
        self.gyro_ax.set_title('Giroscopio (°/s)', color='#7fd6ff', fontsize=14, fontweight='bold')
        self.gyro_ax.set_xlabel('Tiempo (s)', color='#7fd6ff')
        self.gyro_ax.set_ylabel('Velocidad Angular (°/s)', color='#7fd6ff')
        self.gyro_ax.tick_params(colors='#7fd6ff')
        self.gyro_ax.grid(True, alpha=0.3, color='#2a4d6c')
        
        if len(time_relative) > 0:
            self.gyro_ax.plot(time_relative, self.gyro_x_data, color='#ff6b6b', label='X', linewidth=2)
            self.gyro_ax.plot(time_relative, self.gyro_y_data, color='#4ecdc4', label='Y', linewidth=2)
            self.gyro_ax.plot(time_relative, self.gyro_z_data, color='#45b7d1', label='Z', linewidth=2)
            self.gyro_ax.legend(loc='upper right', facecolor='#1b3957', edgecolor='#2a4d6c')
        
        # Ajustar layout y redibujar
        self.graph_fig.tight_layout()
        self.graph_canvas.draw()

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
        # Caras del cubo con transparencia y cian
        self.ax.plot_surface(X, Y, np.full_like(X, 0.5), alpha=0.2, color='#7fd6ff')
        self.ax.plot_surface(X, Y, np.full_like(X, -0.5), alpha=0.2, color='#7fd6ff')
        self.ax.plot_surface(X, np.full_like(X, 0.5), Y, alpha=0.2, color='#7fd6ff')
        self.ax.plot_surface(X, np.full_like(X, -0.5), Y, alpha=0.2, color='#7fd6ff')
        self.ax.plot_surface(np.full_like(X, 0.5), X, Y, alpha=0.2, color='#7fd6ff')
        self.ax.plot_surface(np.full_like(X, -0.5), X, Y, alpha=0.2, color='#7fd6ff')
        # Ejes en cian
        self.ax.plot([0, 0], [0, 0], [-0.7, 0.7], color='#7fd6ff', linewidth=2)
        self.ax.plot([0, 0.7], [0, 0], [0, 0], color='#7fd6ff', linewidth=2)
        self.ax.plot([0, 0], [0, 0.7], [0, 0], color='#7fd6ff', linewidth=2)
        # Fondo y ejes
        self.fig.patch.set_facecolor('#1b3957')
        self.ax.set_facecolor('#1b3957')
        self.ax.set_xlim([-1,1])
        self.ax.set_ylim([-1,1])
        self.ax.set_zlim([-1,1])
        self.ax.set_box_aspect([1,1,1])
        self.ax.axis('off')
        # dentro de init_cube(), sustituyendo tu view_init actual:
        self.ax.view_init(elev=self.pitch,
                  azim=self.yaw,
                  roll=self.roll)

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
                pixmap = QPixmap.fromImage(qt_img).scaled(self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.video_label.setPixmap(pixmap)
                self.last_img = img
            # Actualiza telemetría con datos reales del IMU
            self.bat_bar.setValue(90)
            self.temp_label.setText("Temp: 25.0 °C")
            self.alt_label.setText("Altitud: 123 m")
            self.state_label.setText("Estado: En vuelo")
            
            # Actualiza valores de sensores en el dashboard
            accel_magnitude = math.sqrt(ax_val**2 + ay_val**2 + az_val**2)
            self.accel_value.setText(f"{accel_magnitude:.2f}")
            self.altitude_value.setText("316")  # Puedes actualizar con datos reales de altitud
            self.pressure_value.setText("987.4")  # Puedes actualizar con datos reales de presión
            # --- Datos para mini-grafica:
            now = time.time()
            self.mini_time.append(now)
            self.mini_accel.append(math.sqrt(ax_val**2 + ay_val**2 + az_val**2))

            # Limitar tamaño de la ventana
            if len(self.mini_time) > self.mini_max_pts:
                self.mini_time.pop(0)
                self.mini_accel.pop(0)

            # Convertir a tiempo relativo
            t_rel = [t - self.mini_time[0] for t in self.mini_time]

            # Dibujar mini-grafica
            self.mini_ax.clear()
            self.mini_ax.set_facecolor('#1b3957')
            self.mini_ax.plot(t_rel, self.mini_accel, linewidth=1, label='|a| (g)')
            self.mini_ax.set_xticks([])
            self.mini_ax.set_yticks([])
            self.mini_ax.legend(loc='upper right', facecolor='#1b3957', edgecolor='#2a4d6c', fontsize=8)
            self.mini_fig.tight_layout()
            self.mini_canvas.draw()

            # Actualiza las gráficas con los datos del IMU (solo si estamos en la pestaña de gráficas)
            if self.tab_widget.currentIndex() == 1:  # Pestaña de gráficas
                self.update_graphs(ax_val, ay_val, az_val, gx_val, gy_val, gz_val)
        except Exception as e:
            print("Error en update_data:", e)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    window = MainWindow()
    window.show()
    app.exec_()