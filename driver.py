import os
import serial
import time
import numpy as np
import pyvjoy
import paramiko
import ast
import threading
import select
from colorama import init, Fore

# === CONFIG ===
SERIAL_PORT = 'COM7'       # Replace with your Arduino COM port
BAUD_RATE = 115200         # Match your Arduino sketch
CALIBRATION_DURATION = 5   # Seconds to rotate the wheel for calibration
SENSITIVITY = 0.3           # Adjust how responsive the wheel is
PI_HOST = "raspihole.local"         # or use IP like "192.168.0.100"
PI_USER = "ukii"
PI_PASSWORD = "amogus"             # Or use SSH key
REMOTE_SCRIPT = "/home/ukii/pedale/main.py"

# === CONNECT SERIAL ===
print("Connecting to serial...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Wait for Arduino to reboot
print("Connected.")

# === INIT CALIBRATION ===
axis_labels = ["Yaw", "Pitch", "Roll"]
max_idx = 0  # gyro axis to use

# Shared angle from serial
ypr_angle = 0.0

gas = bremse = kupplung = 0.0
smoothed_gas = smoothed_bremse = smoothed_kupplung = 0.0

def smooth(prev, new, factor):
    return prev * factor + new * (1 - factor)

def stream_pedal_data(lock):
    global gas, bremse, kupplung
    global smoothed_gas, smoothed_bremse, smoothed_kupplung

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        ssh.connect(PI_HOST, username=PI_USER, password=PI_PASSWORD)
        stdin, stdout, stderr = ssh.exec_command(f"python3 -u {REMOTE_SCRIPT}", get_pty=True)

        channel = stdout.channel
        channel.settimeout(0.0)

        buffer = ""
        while True:
            if channel.recv_ready():
                chunk = channel.recv(1024).decode("utf-8")
                buffer += chunk
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    try:
                        values = ast.literal_eval(line.strip())
                        if isinstance(values, list) and len(values) == 3:
                            with lock:
                                gas, bremse, kupplung = values
                                smoothed_gas = smooth(smoothed_gas, gas, PEDAL_SMOOTHING)
                                smoothed_bremse = smooth(smoothed_bremse, bremse, PEDAL_SMOOTHING)
                                smoothed_kupplung = smooth(smoothed_kupplung, kupplung, PEDAL_SMOOTHING)
                    except:
                        continue
            time.sleep(0.001)

    except Exception as e:
        print("SSH error:", e)
    finally:
        ssh.close()

# === SERIAL TRACKER ===
def stream_serial_data():
    global ypr_angle
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if line.startswith("[") and line.endswith("]"):
                ypr = list(map(float, line[1:-1].split(",")))
                if len(ypr) > max_idx:
                    ypr_angle = ypr[max_idx] * 180
        except Exception as e:
            print("Serial error:", e)
        time.sleep(0.001)

# === INIT THREADS ===
lock = threading.Lock()
threading.Thread(target=stream_pedal_data, args=(lock,), daemon=True).start()
threading.Thread(target=stream_serial_data, daemon=True).start()

# === WHEEL TRACKER CLASS ===
class WheelTracker:
    def __init__(self, sensitivity=1.0):
        self.sensitivity = sensitivity
        self.total_rotation = 0.0
        self.last_angle = None

    def update(self, current_angle):
        if self.last_angle is None:
            self.last_angle = current_angle
            return

        delta = current_angle - self.last_angle

        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360

        self.total_rotation += delta * self.sensitivity
        self.total_rotation = max(min(self.total_rotation, 179), -179)

        self.last_angle = current_angle

    def get_total_rotation(self):
        return self.total_rotation

    def reset(self):
        self.total_rotation = 0.0
        self.last_angle = None

# === INIT JOYSTICK ===
j = pyvjoy.VJoyDevice(1)

# Initialize wheel tracker for selected axis
wheel_tracker = WheelTracker(sensitivity=SENSITIVITY)

def map_to_axis(value):
    value = value + 180
    normalized = (value % 360) / 360
    return int(normalized * 32767)

def map_to_axis_two(value):
    value += 10
    value = min(value, 100)
    value = max(value, 0)
    value /= 100
    value = 1 - value
    return int(value * 32767)

print(Fore.RESET + "Sending joystick input... (Ctrl+C to stop)")

time.sleep(3)
last = time.time()
i = 0
try:
    while True:
        i+=1
        time.sleep(0.001)
        fps = 1 / (time.time() - last + 1e-10)
        if i % 1000 == 0:
            os.system("cls")
            print(Fore.GREEN + f"FPS: {fps:.2f}")
        last = time.time()

        wheel_tracker.update(ypr_angle)
        rot = wheel_tracker.get_total_rotation()
        val = map_to_axis(rot)
        j.set_axis(pyvjoy.HID_USAGE_X, val)  # Wheel → X axis

        with lock:
            j.set_axis(pyvjoy.HID_USAGE_Y, map_to_axis_two(smoothed_gas))       # Gas → Y axis
            j.set_axis(pyvjoy.HID_USAGE_Z, map_to_axis_two(smoothed_bremse))    # Brake → Z axis
            j.set_axis(pyvjoy.HID_USAGE_RX, map_to_axis_two(smoothed_kupplung)) # Clutch → RX axis


except KeyboardInterrupt:
    print("\nStopped.")
    ser.close()
