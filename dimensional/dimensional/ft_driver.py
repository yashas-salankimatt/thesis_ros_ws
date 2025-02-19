import serial
import time
import re
import dash
from dash import dcc, html
import plotly.graph_objs as go
import threading
from tabulate import tabulate

# show the dash app
show_dash = True

class SensorData:
    def __init__(self, moving_average_window=3, calibration_window=100):
        self.sensor_data_window = []
        self.sensor_moving_averages = []
        self.calibration_data = []
        self.sensor_calibrated_averages = []
        self.moving_average_window = moving_average_window
        self.calibration_window = calibration_window
        self.calibrated = False
    
    def process_data(self, values):
        # check if any of values are invalid
        if any(x < 9000 or x > 21000 for x in values or len(values) != 16):
            print("Invalid values:", values)
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

class SerialReader:
    def __init__(self, sensor_data, port="/dev/ttyUSB0", baudrate=115200, timeout=1):
        self.sensor_data = sensor_data
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.connect_serial()

    def connect_serial(self):
        """Attempt to connect to the serial port with retries."""
        while True:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                time.sleep(2)  # Allow time for the connection to establish
                print(f"Connected to {self.port}")
                break
            except serial.SerialException as e:
                print(f"Failed to connect: {e}. Retrying in 3 seconds...")
                time.sleep(3)

    def read_data(self):
        """Reads data only if available."""
        if self.ser is None or not self.ser.is_open:
            print("Reconnecting to serial port...")
            self.connect_serial()

        try:
            if self.ser.in_waiting > 0:  # Only read if data is available
                line = self.ser.readline().decode('utf-8').strip('\r\n,')
                if line:
                    # try:
                        # values = [int(pair.split(":")[1]) for pair in re.split(r'[\t\n\r,]+', line)]
                    values = [int(pair) for pair in re.split(r'[\t\n\r,]+', line)]
                    self.sensor_data.process_data(values)
                    # except Exception as e:
                    #     print("Error parsing line:", line, "Error:", e)
        except (serial.SerialException, OSError) as e:
            print("Serial error:", e)
            self.ser.close()
            self.ser = None  # Reset connection
            # time.sleep(1)


    def close(self):
        """Closes the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")

# Initialize sensor data storage
sensor_data = SensorData()
serial_reader = SerialReader(sensor_data, port="/dev/ttyACM0")  # Change to your serial port

# Start reading serial data in a separate thread
def serial_thread():
    while True:
        serial_reader.read_data()
        time.sleep(0.04)

thread = threading.Thread(target=serial_thread, daemon=True)
thread.start()

def print_thread():
    while True:
        # print the moving averages as integers, formatted as a table
        print(tabulate([[int(x) for x in sensor_data.sensor_moving_averages]], headers=["Sensor 1", "Sensor 2", "Sensor 3", "Sensor 4", "Sensor 5", "Sensor 6", "Sensor 7", "Sensor 8", "Sensor 9", "Sensor 10", "Sensor 11", "Sensor 12", "Sensor 13", "Sensor 14", "Sensor 15", "Sensor 16"]))
        time.sleep(0.05)

print_thread = threading.Thread(target=print_thread, daemon=True)
print_thread.start()

if show_dash:
    # Dash App for Real-time Plotting
    app = dash.Dash(__name__)

    app.layout = html.Div([
        html.H1("Real-time Sensor Data"),
        dcc.Graph(id="sensor-graph"),
        dcc.Interval(id="interval-component", interval=100, n_intervals=0)  # Update every 100ms
    ])

    @app.callback(
        dash.dependencies.Output("sensor-graph", "figure"),
        [dash.dependencies.Input("interval-component", "n_intervals")]
    )
    def update_graph(n):
        """Fetches sensor data and updates the plot."""
        if not sensor_data.sensor_data_window:
            return go.Figure()

        # Convert data to a structured format
        num_sensors = len(sensor_data.sensor_data_window[0])
        x_vals = list(range(len(sensor_data.sensor_data_window)))
        
        fig = go.Figure()
        for i in range(num_sensors):
            y_vals = [row[i] for row in sensor_data.sensor_data_window]
            fig.add_trace(go.Scatter(x=x_vals, y=y_vals, mode="lines", name=f"Sensor {i+1} Raw"))
        
        if sensor_data.sensor_moving_averages:
            for i in range(num_sensors):
                y_vals_avg = [sum(row[i] for row in sensor_data.sensor_data_window[-sensor_data.moving_average_window:]) / sensor_data.moving_average_window for _ in sensor_data.sensor_data_window]
                fig.add_trace(go.Scatter(x=x_vals, y=y_vals_avg, mode="lines", name=f"Sensor {i+1} Avg", line=dict(dash='dash')))

        fig.update_layout(title="Sensor Readings", xaxis_title="Time", yaxis_title="Value")
        return fig

if __name__ == "__main__":
    if show_dash:
        app.run_server(debug=True)
    else:
        while True:     # TODO: fix delay in print data when dash is not shown
            time.sleep(0.01)
