import serial
import time
import re
import dash
from dash import dcc, html
import plotly.graph_objs as go
import threading
from tabulate import tabulate
import math

# show the dash app
show_dash = True

class SensorData:
    def __init__(self, moving_average_window=3, calibration_window=100):
        self.sensor_data_window = []
        self.sensor_moving_averages = []
        self.calibration_data = []
        self.sensor_calibrated_averages = []
        self.cluster_displacements = {"Middle": [], "Top": [], "Bottom Right": [], "Bottom Left": []}
        self.cluster_angles = [0, 0, 240, 120]
        self.magnet_dist = .025
        self.cluster_distances = [0, self.magnet_dist, self.magnet_dist, self.magnet_dist]
        self.cluster_torques = {"Middle": [], "Top": [], "Bottom Right": [], "Bottom Left": []}
        self.forces = []
        self.torques = []
        self.moving_average_window = moving_average_window
        self.calibration_window = calibration_window
        self.calibrated = False
    
    def process_data(self, values):
        if any(x < 9000 or x > 21000 for x in values) or len(values) != 16:
            # print("Invalid values:", values)
            return
        self.sensor_data_window.append(values)
        if len(self.sensor_data_window) > self.calibration_window:
            self.sensor_data_window.pop(0)
        if len(self.sensor_data_window) == self.calibration_window and not self.calibrated:
            self.calibration_data = self.sensor_data_window.copy()
            self.sensor_calibrated_averages = [sum(x) / len(x) for x in zip(*self.calibration_data)]
            self.calibrated = True
            print("Calibrated Averages:", self.sensor_calibrated_averages)
        if len(self.sensor_data_window) > self.moving_average_window:
            self.sensor_moving_averages = [sum(x) / len(x) for x in zip(*self.sensor_data_window[-self.moving_average_window:])]
            if self.calibrated:
                self.sensor_moving_averages = [x - self.sensor_calibrated_averages[i] for i, x in enumerate(self.sensor_moving_averages)]
                self.calculate_displacements()

    def calculate_displacements(self):
        if len(self.sensor_moving_averages) != 16:
            return
        clusters = {
            "Middle": self.sensor_moving_averages[0:4],
            "Top": self.sensor_moving_averages[4:8],
            "Bottom Right": self.sensor_moving_averages[8:12],
            "Bottom Left": self.sensor_moving_averages[12:]
        }
        force_x = 0
        force_y = 0
        force_z = 0
        torque_x = 0
        torque_y = 0
        torque_z = 0
        for i, (name, cluster) in enumerate(clusters.items()):
            local_x_displacement = cluster[1] - cluster[3]
            local_y_displacement = cluster[0] - cluster[2]
            local_z_displacement = sum(cluster) / len(cluster)
            x_displacement = local_x_displacement * math.cos(math.radians(self.cluster_angles[i])) - local_y_displacement * math.sin(math.radians(self.cluster_angles[i]))
            y_displacement = local_x_displacement * math.sin(math.radians(self.cluster_angles[i])) + local_y_displacement * math.cos(math.radians(self.cluster_angles[i]))
            force_x += x_displacement
            force_y += y_displacement
            force_z += local_z_displacement
            position_vector = []
            position_vector.append(self.cluster_distances[i] * -math.sin(math.radians(self.cluster_angles[i])))
            position_vector.append(self.cluster_distances[i] * math.cos(math.radians(self.cluster_angles[i])))
            position_vector.append(0)
            # print(name, position_vector)
            local_torque_x = position_vector[1] * force_z - position_vector[2] * force_y
            local_torque_y = position_vector[2] * force_x - position_vector[0] * force_z
            local_torque_z = position_vector[0] * force_y - position_vector[1] * force_x
            torque_x += local_torque_x
            torque_y += local_torque_y
            torque_z += local_torque_z
            self.cluster_displacements[name].append([x_displacement, y_displacement, local_z_displacement])
            if len(self.cluster_displacements[name]) > 100:
                self.cluster_displacements[name].pop(0)
            self.cluster_torques[name].append([torque_x, torque_y, torque_z])
            if len(self.cluster_torques[name]) > 100:
                self.cluster_torques[name].pop(0)
        self.forces.append([force_x, force_y, force_z])
        if len(self.forces) > 100:
            self.forces.pop(0)
        self.torques.append([torque_x, torque_y, torque_z])
        if len(self.torques) > 100:
            self.torques.pop(0)
        # print(tabulate([[torque_x, torque_y, torque_z]], headers=["Torque X", "Torque Y", "Torque Z"]))

class SerialReader:
    def __init__(self, sensor_data, port="/dev/ttyUSB0", baudrate=115200, timeout=1):
        self.sensor_data = sensor_data
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.connect_serial()

    def connect_serial(self):
        while True:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                time.sleep(2)
                print(f"Connected to {self.port}")
                break
            except serial.SerialException as e:
                print(f"Failed to connect: {e}. Retrying in 3 seconds...")
                time.sleep(3)

    def read_data(self):
        if self.ser is None or not self.ser.is_open:
            print("Reconnecting to serial port...")
            self.connect_serial()
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip('\r\n,')
                if line:
                    values = [int(pair) for pair in re.split(r'[\t\n\r,]+', line)]
                    self.sensor_data.process_data(values)
        except (serial.SerialException, OSError) as e:
            print("Serial error:", e)
            self.ser.close()
            self.ser = None

sensor_data = SensorData()
serial_reader = SerialReader(sensor_data, port="/dev/ttyACM0")

def serial_thread():
    while True:
        serial_reader.read_data()
        time.sleep(0.05)

thread = threading.Thread(target=serial_thread, daemon=True)
thread.start()

if show_dash:
    app = dash.Dash(__name__)
    cluster_names = ["Middle", "Top", "Bottom Right", "Bottom Left"]
    app.layout = html.Div([
        html.H1("Real-time Sensor Data & Cluster Displacements"),
        dcc.Graph(id="raw-sensor-graph"),
        html.Div([
            html.Div([dcc.Graph(id=f"{name}-graph")], style={'width': '48%', 'display': 'inline-block'}) for name in cluster_names
        ], style={'display': 'flex', 'flex-wrap': 'wrap'}),
        html.Div([
            html.Div([dcc.Graph(id=f"{name}-torque-graph")], style={'width': '48%', 'display': 'inline-block'}) for name in cluster_names
        ], style={'display': 'flex', 'flex-wrap': 'wrap'}),
        html.Div([
            html.Div([dcc.Graph(id=f"force-graph")], style={'width': '48%', 'display': 'inline-block'}),
            html.Div([dcc.Graph(id=f"torque-graph")], style={'width': '48%', 'display': 'inline-block'}),
        ], style={'display': 'flex', 'flex-wrap': 'wrap'}),
        dcc.Interval(id="interval-component", interval=100, n_intervals=0)
    ])
    
    @app.callback(
        dash.dependencies.Output("raw-sensor-graph", "figure"),
        [dash.dependencies.Input("interval-component", "n_intervals")]
    )
    def update_raw_sensor_graph(n):
        if not sensor_data.sensor_data_window:
            return go.Figure()
        x_vals = list(range(len(sensor_data.sensor_data_window)))
        fig = go.Figure()
        for i in range(16):
            y_vals = [row[i] for row in sensor_data.sensor_data_window]
            fig.add_trace(go.Scatter(x=x_vals, y=y_vals, mode="lines", name=f"Sensor {i+1}"))
        fig.update_layout(title="Raw Sensor Data", xaxis_title="Time", yaxis_title="Value")
        return fig
    
    for name in cluster_names:
        @app.callback(
            dash.dependencies.Output(f"{name}-graph", "figure"),
            [dash.dependencies.Input("interval-component", "n_intervals")]
        )
        def update_displacement_graph(n, cluster=name):
            if not sensor_data.cluster_displacements[cluster]:
                return go.Figure()
            x_vals = list(range(len(sensor_data.cluster_displacements[cluster])))
            fig = go.Figure()
            displacements = list(zip(*sensor_data.cluster_displacements[cluster]))
            for dim, axis in enumerate(["X", "Y", "Z"]):
                fig.add_trace(go.Scatter(x=x_vals, y=displacements[dim], mode="lines", name=f"{axis} Displacement"))
            fig.update_layout(title=f"{cluster} Cluster Displacements", xaxis_title="Time", yaxis_title="Displacement")
            return fig
    
    for name in cluster_names:
        @app.callback(
            dash.dependencies.Output(f"{name}-torque-graph", "figure"),
            [dash.dependencies.Input("interval-component", "n_intervals")]
        )
        def update_torque_graph(n, cluster=name):
            if not sensor_data.cluster_torques[cluster]:
                return go.Figure()
            x_vals = list(range(len(sensor_data.cluster_torques[cluster])))
            fig = go.Figure()
            torques = list(zip(*sensor_data.cluster_torques[cluster]))
            for dim, axis in enumerate(["X", "Y", "Z"]):
                fig.add_trace(go.Scatter(x=x_vals, y=torques[dim], mode="lines", name=f"{axis} Torque"))
            fig.update_layout(title=f"{cluster} Cluster Torques", xaxis_title="Time", yaxis_title="Torque")
            return fig
    
    @app.callback(
        dash.dependencies.Output("force-graph", "figure"),
        [dash.dependencies.Input("interval-component", "n_intervals")]
    )
    def update_force_graph(n):
        if not sensor_data.forces:
            return go.Figure()
        x_vals = list(range(len(sensor_data.forces)))
        fig = go.Figure()
        forces = list(zip(*sensor_data.forces))
        for dim, axis in enumerate(["X", "Y", "Z"]):
            fig.add_trace(go.Scatter(x=x_vals, y=forces[dim], mode="lines", name=f"{axis} Force"))
        fig.update_layout(title="Force", xaxis_title="Time", yaxis_title="Force")
        return fig
    
    @app.callback(
        dash.dependencies.Output("torque-graph", "figure"),
        [dash.dependencies.Input("interval-component", "n_intervals")]
    )
    def update_torque_graph(n):
        if not sensor_data.torques:
            return go.Figure()
        x_vals = list(range(len(sensor_data.torques)))
        fig = go.Figure()
        torques = list(zip(*sensor_data.torques))
        for dim, axis in enumerate(["X", "Y", "Z"]):
            fig.add_trace(go.Scatter(x=x_vals, y=torques[dim], mode="lines", name=f"{axis} Torque"))
        fig.update_layout(title="Torque", xaxis_title="Time", yaxis_title="Torque")
        return fig
    
    app.run_server(debug=True)