import smbus2
import time
import math
from mpu6050 import mpu6050

MUX_ADDR = 0x70
MUX_CHANNELS = [0, 1, 2]
MPU_ADDR = 0x68
bus = smbus2.SMBus(1)

def select_mux_channel(channel):
    try:
        bus.write_byte(MUX_ADDR, 1 << channel)
        time.sleep(0.01)  # 10ms delay for precision
    except Exception as e:
        print(f"Multiplexer error: {e}")

def safe_read(read_fn, retries=3):
    for _ in range(retries):
        try:
            return read_fn()
        except OSError:
            time.sleep(0.005)
    return None

def get_pitch(accel):
    # Calculate pitch from accelerometer data (in degrees)
    ax = accel['x']
    ay = accel['y']
    az = accel['z']
    pitch = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
    return pitch

# Initialize sensor objects for each channel
sensors = []
for channel in MUX_CHANNELS:
    select_mux_channel(channel)
    sensors.append(mpu6050(MPU_ADDR))

# At startup, get the initial reading for each sensor as the maximum (100 position)
max_vals = []
min_vals = []
for idx, channel in enumerate(MUX_CHANNELS):
    select_mux_channel(channel)
    accel = safe_read(sensors[idx].get_accel_data)
    initial = get_pitch(accel)
    # Invert the pitch for the third sensor (index 2)
    if idx == 2:
        initial = -initial
    max_vals.append(initial)
    min_vals.append(initial)

# Main loop: Read each sensor, update min, and normalize output from 0 to 100
while True:
    normalized_positions = []
    for idx, channel in enumerate(MUX_CHANNELS):
        select_mux_channel(channel)
        accel = safe_read(sensors[idx].get_accel_data)
        if accel is None:
            current = max_vals[idx]  # use startup value if read fails
        else:
            current = get_pitch(accel)
            if idx == 2:  # Invert third sensor reading
                current = -current

        # Update the minimum value observed
        if current < min_vals[idx]:
            min_vals[idx] = current

        # Calculate normalized position:
        # If current equals startup value (max_vals[idx]), output 100.
        # If it equals min_vals[idx], output 0.
        diff = max_vals[idx] - min_vals[idx]
        if diff == 0:
            norm = 100
        else:
            norm = ((current - min_vals[idx]) / diff) * 105

        normalized_positions.append(round(norm, 2))
    print(normalized_positions)
    time.sleep(0.01)
