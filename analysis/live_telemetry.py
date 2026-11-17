import sys
import os
import glob
import struct
import pandas as pd
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# Configuración
LOG_DIR = "/home/sysmic/log"
STRUCT_FORMAT = "<BHfffffffB"
STRUCT_SIZE = 32

class LiveTelemetryPlotter(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Telemetría STM32 - Tiempo Real")
        self.resize(1100, 800)

        # Contenedor principal de PyQtGraph
        self.win = pg.GraphicsLayoutWidget(show=True)
        self.setCentralWidget(self.win)

        # Configurar Subplots
        self.p_pos = self.win.addPlot(title="Posición Estimada (x, y) [m]")
        self.p_pos.addLegend()
        self.p_pos.showGrid(x=True, y=True)
        self.curve_x = self.p_pos.plot(pen=pg.mkPen('b', width=2), name="x_est")
        self.curve_y = self.p_pos.plot(pen=pg.mkPen('r', width=2), name="y_est")

        self.win.nextRow()
        self.p_vel = self.win.addPlot(title="Velocidades (vx, vy) [m/s]")
        self.p_vel.addLegend()
        self.p_vel.showGrid(x=True, y=True)
        self.curve_vx = self.p_vel.plot(pen=pg.mkPen('g', width=2), name="vx_est")
        self.curve_vy = self.p_vel.plot(pen=pg.mkPen('c', width=2), name="vy_est")

        self.win.nextRow()
        self.p_ang = self.win.addPlot(title="Variables Angulares (theta, omega, gyro_z)")
        self.p_ang.addLegend()
        self.p_ang.showGrid(x=True, y=True)
        self.curve_theta = self.p_ang.plot(pen=pg.mkPen('m', width=2), name="theta_est")
        self.curve_omega = self.p_ang.plot(pen=pg.mkPen('y', width=2), name="omega_est")
        self.curve_gyro = self.p_ang.plot(pen=pg.mkPen('w', width=1, style=QtCore.Qt.DashLine), name="gyro_z")

        # Vincular el Zoom/Pan del eje X entre todas las gráficas
        self.p_vel.setXLink(self.p_pos)
        self.p_ang.setXLink(self.p_pos)

        # Buffer de almacenamiento en memoria
        self.data_store = {
            'time_sec': [],
            'x_est': [], 'y_est': [],
            'vx_est': [], 'vy_est': [],
            'theta_est': [], 'omega_est': [], 'gyro_z': []
        }
        
        self.last_state = None
        self.last_file = None
        self.file_pos = 0  # Offset de lectura en el CSV

        # Timer para polling rápido (actualiza la UI a ~30 FPS)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_telemetry)
        self.timer.start(33)  # 33 ms = ~30 Hz

    def get_latest_csv(self):
        """Encuentra el archivo CSV más reciente en /home/sysmic/log"""
        csv_files = glob.glob(os.path.join(LOG_DIR, "*.csv"))
        if not csv_files:
            return None
        return max(csv_files, key=os.path.getmtime)

    def unpack_telemetry(self, raw_bytes):
        if len(raw_bytes) != 32:
            return None
        unpacked = struct.unpack(STRUCT_FORMAT, raw_bytes)
        return {
            'x_est': unpacked[2],
            'y_est': unpacked[3],
            'vx_est': unpacked[4],
            'vy_est': unpacked[5],
            'theta_est': unpacked[6],
            'omega_est': unpacked[7],
            'gyro_z': unpacked[8],
        }

    def update_telemetry(self):
        latest_csv = self.get_latest_csv()
        if not latest_csv:
            return

        # Si STM32CubeMonitor creó un archivo nuevo, reiniciar lectura
        if latest_csv != self.last_file:
            self.last_file = latest_csv
            self.file_pos = 0
            for k in self.data_store:
                self.data_store[k].clear()
            self.last_state = None
            print(f"Monitoreando nuevo archivo: {latest_csv}")

        # Leer solo las nuevas líneas agregadas al archivo
        try:
            with open(latest_csv, 'r') as f:
                if self.file_pos == 0:
                    header = f.readline()  # Saltar encabezado
                    self.file_pos = f.tell()
                else:
                    f.seek(self.file_pos)

                new_lines = f.readlines()
                self.file_pos = f.tell()

            if not new_lines:
                return

            # Procesar nuevas líneas
            for line in new_lines:
                parts = line.strip().split(',')
                if len(parts) < 33:
                    continue

                try:
                    # Extraer bytes telemetry_buf[0..31]
                    raw_bytes = bytes([int(float(parts[i+1])) for i in range(32)])
                    data = self.unpack_telemetry(raw_bytes)

                    if data:
                        # Filtrar muestras duplicadas consecutivas
                        current_state = (data['x_est'], data['y_est'], data['vx_est'], 
                                         data['vy_est'], data['theta_est'], data['omega_est'], data['gyro_z'])
                        
                        if current_state != self.last_state:
                            self.last_state = current_state

                            # Reconstruir tiempo incremental a 1 kHz (1ms)
                            t_sec = len(self.data_store['time_sec']) * 0.001
                            
                            self.data_store['time_sec'].append(t_sec)
                            for key in data:
                                self.data_store[key].append(data[key])
                except (ValueError, IndexError):
                    continue

            # Actualizar curvas en pantalla (Ventana deslizante de los últimos 10.000 puntos para rendimiento)
            if len(self.data_store['time_sec']) > 0:
                t = np.array(self.data_store['time_sec'][-10000:])
                
                self.curve_x.setData(t, np.array(self.data_store['x_est'][-10000:]))
                self.curve_y.setData(t, np.array(self.data_store['y_est'][-10000:]))
                self.curve_vx.setData(t, np.array(self.data_store['vx_est'][-10000:]))
                self.curve_vy.setData(t, np.array(self.data_store['vy_est'][-10000:]))
                self.curve_theta.setData(t, np.array(self.data_store['theta_est'][-10000:]))
                self.curve_omega.setData(t, np.array(self.data_store['omega_est'][-10000:]))
                self.curve_gyro.setData(t, np.array(self.data_store['gyro_z'][-10000:]))

        except Exception as e:
            pass

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    main = LiveTelemetryPlotter()
    main.show()
    sys.exit(app.exec_())