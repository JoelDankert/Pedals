import serial
import time
import numpy as np
import pyvjoy
import paramiko
import ast
import threading
from colorama import init, Fore

# === CONFIG ===
SERIAL_PORT = 'COM7'       # Replace with your Arduino COM port
BAUD_RATE = 115200         # Match your Arduino sketch
CALIBRATION_DURATION = 5   # Seconds to rotate the wheel for calibration
SENSITIVITY = 0.5         # Adjust how responsive the wheel is
# ==============
PI_HOST = "raspihole.local"         # or use IP like "192.168.0.100"
PI_USER = "ukii"
PI_PASSWORD = "amogus"       # Or use SSH key
REMOTE_SCRIPT = "/home/ukii/pedale/main.py"
# ==============


# === CONNECT SERIAL ===
print("Connecting to serial...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Wait for Arduino to reboot
print("Connected.")

# === INIT CALIBRATION ===
prev_ypr = None
diff_sums = np.zeros(3)  # [Yaw, Pitch, Roll]
axis_labels = ["Yaw", "Pitch", "Roll"]
max_idx = 0 # gyro axis to use

gas = bremse = kupplung = 0.0




# === PEDAL TRACKER ===
def stream_pedal_data(lock):
    global gas, bremse, kupplung

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        ssh.connect(PI_HOST, username=PI_USER, password = PI_PASSWORD)
        stdin, stdout, stderr = ssh.exec_command("python3 -u /home/ukii/pedale/main.py")


        for line in iter(stdout.readline, ""):
            try:
                values = ast.literal_eval(line.strip())
                if isinstance(values, list) and len(values) == 3:
                    with lock:
                        gas, bremse, kupplung = values
            except:
                continue

    except Exception as e:
        print("SSH error:", e)
    finally:
        ssh.close()


lock = threading.Lock()
threading.Thread(target=stream_pedal_data, args=(lock,), daemon=True).start()


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
        if self.total_rotation > 179:
            self.total_rotation = 179
        elif self.total_rotation < -179:
            self.total_rotation = -179

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
    # Normalize to [0, 1], then scale to [0, 32767]
    # You can allow for multiple rotations by expanding the range
    value = value + 180
    normalized = (value % 360) / 360  # Modulo allows rollover
    #normalized = 1 - normalized # invert
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

try:
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("[") and line.endswith("]"):
            try:
                ypr = list(map(float, line[1:-1].split(",")))
                angle_deg = ypr[max_idx] * 180  # Convert from [-1,1] to [-180,180]
                wheel_tracker.update(angle_deg)
                rot = wheel_tracker.get_total_rotation()
                val = map_to_axis(rot)
                j.set_axis(pyvjoy.HID_USAGE_X, val)  # Wheel → X axis

                # Safely update pedal axes using shared lock
                with lock:
                    j.set_axis(pyvjoy.HID_USAGE_Y, map_to_axis_two(gas))       # Gas → Y axis
                    j.set_axis(pyvjoy.HID_USAGE_Z, map_to_axis_two(bremse))    # Brake → Z axis
                    j.set_axis(pyvjoy.HID_USAGE_RX, map_to_axis_two(kupplung)) # Clutch → RX axis

            except Exception as e:
                print("check your connection...", e)
                continue
        else:
            print("check your connection...")
except KeyboardInterrupt:
    print("\nStopped.")
    ser.close()

