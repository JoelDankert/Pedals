import os
import serial
import time
import numpy as np
import pyvjoy
import paramiko
import ast
import threading
import sys
import select
from colorama import init, Fore

os.system('')

# === CONFIG ===
SERIAL_PORT = 'COM7'       # Replace with your Arduino COM port
BAUD_RATE = 115200         # Match your Arduino sketch
CALIBRATION_DURATION = 5   # Seconds to rotate the wheel for calibration
SENSITIVITY = 0.3           # Adjust how responsive the wheel is
PI_HOST = "raspihole.local"         # or use IP like "192.168.0.100"
PI_USER = "ukii"
PI_PASSWORD = "amogus"             # Or use SSH key
REMOTE_SCRIPT = "/home/ukii/pedale/main.py"
PEDAL_SMOOTHING = 0.8  # 0 = no smoothing, 1 = full smoothing (recommended range: 0.1 - 0.5)
deadzone = 0.05
button_states = [0,0,0]
button_event_states = {
    0: {'start_time': 0, 'long_active': False, 'short_end_time': 0},
    1: {'start_time': 0, 'long_active': False, 'short_end_time': 0},
    2: {'start_time': 0, 'long_active': False, 'short_end_time': 0}
}

prev_button_states = [0, 0, 0]

LONG_PRESS_THRESHOLD = 200
SHORT_PRESS_DURATION = 300

tilt_start_angle = None

def render_progress_bar(name: str, fill: float, marker: float, bar_width: int = 30):
    fill = max(0, min(100, fill))
    marker = max(0, min(100, marker))

    fill_length = int(bar_width * fill / 100)
    marker_pos = int(bar_width * marker / 100)

    GREEN_BG = '\033[42m'
    WHITE_BG = '\033[47m'
    RESET = '\033[0m'

    bar = ""
    for i in range(bar_width):
        if i == marker_pos:
            bg = GREEN_BG if i < fill_length else WHITE_BG
            bar += f"{bg}|{RESET}"
        elif i < fill_length:
            bar += f"{GREEN_BG} {RESET}"
        else:
            bar += f"{WHITE_BG} {RESET}"

    sys.stdout.write(f"\r{bar} {name}\n")
    sys.stdout.flush()
  


# === CONNECT SERIAL ===
print("Connecting to serial...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Wait for Arduino to reboot
print("Connected.")

# === INIT CALIBRATION ===
axis_labels = ["Yaw", "Pitch", "Roll"]
rotindex = 0  # gyro axis to use for rotation
tiltindex = 1  # gyro axis to access tilt (pitch), not used directly


# Shared angle from serial
rot_angle = 0.0
tilt_angle = 0.0

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
                                kupplung, bremse, gas = values
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
    global ypr_angle, tilt_angle, button_states
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            # Expecting a line like "[yaw,pitch,roll],[btn1,btn2,btn3]"
            if line.startswith("[") and "],[" in line and line.endswith("]"):
                # Remove the first [ and last ] then split by "],["
                content = line[1:-1]
                parts = content.split("],[")
                if len(parts) == 2:
                    # Parse the orientation values
                    ypr = list(map(float, parts[0].split(",")))
                    # Parse the button states (convert them to integers)
                    buttons = list(map(int, parts[1].split(",")))
                    
                    # Check if the expected indexes are present
                    if len(ypr) > max(rotindex, tiltindex):
                        ypr_angle = ypr[rotindex] * 180  # assuming conversion factor, as before
                        tilt_angle = ypr[tiltindex] * 180  # accessible but unused
                        
                    # Update the button_states global variable
                    button_states = buttons
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

def map_to_axis_two(value): # 100 - 0
    value /= 100
    value = value * (1+deadzone/2) - deadzone  # stretch range, then shift down
    value = max(0.0, min(value, 1.0))  # clamp to 0–1
    value = 1.0 - value  # invert (0 = full press)
    return int(value * 32767)

def process_button_events():
    global prev_button_states
    current_time = time.time() * 1000  # current time in milliseconds
    for i in range(3):
        current_state = button_states[i]  # current reading: 1 if pressed, 0 if released
        
        # Detect rising edge: button just pressed
        if current_state == 1 and prev_button_states[i] == 0:
            button_event_states[i]['start_time'] = current_time
            button_event_states[i]['long_active'] = False
            # Ensure both potential events are off initially
            j.set_button(1 + i, False)  # short press button
            j.set_button(4 + i, False)  # long press button
        
        # While the button is held down
        if current_state == 1:
            duration = current_time - button_event_states[i]['start_time']
            # If we've passed the long press threshold and haven't activated the long event yet
            if duration >= LONG_PRESS_THRESHOLD and not button_event_states[i]['long_active']:
                j.set_button(4 + i, True)  # Activate long press event
                button_event_states[i]['long_active'] = True
        
        # Detect falling edge: button just released
        if current_state == 0 and prev_button_states[i] == 1:
            duration = current_time - button_event_states[i]['start_time']
            if duration < LONG_PRESS_THRESHOLD:
                # For any press shorter than the long press threshold, trigger short press event
                j.set_button(1 + i, True)
                button_event_states[i]['short_end_time'] = current_time + SHORT_PRESS_DURATION
            else:
                # If it was a long press, immediately turn off the long press event
                if button_event_states[i]['long_active']:
                    j.set_button(4 + i, False)
                    button_event_states[i]['long_active'] = False
        
        # Manage the expiration of a short press event:
        # If a short press event is active (tracked via short_end_time) and its duration has passed, turn it off.
        if button_event_states[i]['short_end_time'] and current_time >= button_event_states[i]['short_end_time']:
            j.set_button(1 + i, False)
            button_event_states[i]['short_end_time'] = 0
        
        # Update the previous state for the next iteration
        prev_button_states[i] = current_state


print(Fore.RED + "Starting, dont move please")
time.sleep(5)
print(Fore.RESET + "Sending joystick input... (Ctrl+C to stop)")
print(Fore.GREEN + "HOLD ALL PEDALS FULLY DOWN FOR 1 SEC TO CALIBRATE")
time.sleep(2)
last = time.time()
i = 0

os.system("cls")

for _ in range(7):
    print(" ")  # Reserve space for 7 lines (fps + 4 bars + gaps)

sys.stdout.write('\033[?25l')  # Hide cursor
sys.stdout.flush()

try:
    while True:
        render = False
        i += 1
        time.sleep(0.0005)
        fps = 1 / (time.time() - last + 1e-10)
        if i % 10 == 0: 
            sys.stdout.write('\033[F' * 6)  # Move up 
            sys.stdout.flush()
            render = True
            print(Fore.GREEN + f"FPS: {fps:.2f}")
        last = time.time()

        wheel_tracker.update(ypr_angle)
        rot = wheel_tracker.get_total_rotation()
        val = map_to_axis(rot)
        
        if render:
            render_progress_bar("W", (rot/180+1)*51, 50, 50)
            print("")

        j.set_axis(pyvjoy.HID_USAGE_X, val)  # Wheel → X axis
        
        process_button_events()

        with lock:
            if render:
                render_progress_bar("C", smoothed_kupplung, kupplung, 50)
                render_progress_bar("B", smoothed_bremse, bremse, 50)
                render_progress_bar("A", smoothed_gas, gas, 50)

            j.set_axis(pyvjoy.HID_USAGE_Y, map_to_axis_two(smoothed_gas))       # Gas → Y axis
            j.set_axis(pyvjoy.HID_USAGE_Z, map_to_axis_two(smoothed_bremse))      # Brake → Z axis
            j.set_axis(pyvjoy.HID_USAGE_RX, map_to_axis_two(smoothed_kupplung))    # Clutch → RX axis
            
            

except KeyboardInterrupt:
    print("\nStopped.")
    sys.stdout.write('\033[?25h')  # Show cursor again
    sys.stdout.flush()
    ser.close()
