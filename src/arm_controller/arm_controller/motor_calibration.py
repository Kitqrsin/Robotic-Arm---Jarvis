import time

import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from board import SCL, SDA

# --- SETUP ---
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Define which channels you are using (0 to 5 for a 6-DOF arm)
active_channels = [0, 1, 2, 3, 4, 5]

print("--- CALIBRATION MODE: 90 DEGREES ---")
print("Moving motors to 90°...")
print("Keep this running while you attach your servo horns.")

# List to keep servo objects alive
my_servos = []

for channel in active_channels:
    try:
        # Standard range 500-2500us.
        # If your motor buzzes at limits later, you can tweak this,
        # but for centering, standard is usually fine.
        s = servo.Servo(pca.channels[channel], min_pulse=500, max_pulse=2500)

        # If you have a 360 degree servo for the base,
        # we need to tell Python the range is wider.
        if channel == 0:
            s.actuation_range = 360
            # Note: For a 360 servo, "90" might not be center.
            # Center is usually 180. Change below if needed.

        s.angle = 90
        my_servos.append(s)
        print(f"Channel {channel} -> SET to 90°")

    except Exception as e:
        print(f"Error on channel {channel}: {e}")

print("\n--- HOLDING POSITION ---")
print("Motors are now active. Do not close this window.")
print("Press CTRL+C when finished to release motors.")

try:
    while True:
        time.sleep(1)  # Infinite loop to keep the signal active
except KeyboardInterrupt:
    print("\nReleasing motors...")
    pca.deinit()
