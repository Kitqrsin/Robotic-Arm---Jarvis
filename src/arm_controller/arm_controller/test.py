from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
from adafruit_motor import servo
import time

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# We use servo.Servo for Standard (Positional) servos
servos = [
    servo.Servo(pca.channels[0]),
    servo.Servo(pca.channels[1]),
    servo.Servo(pca.channels[2]),
    servo.Servo(pca.channels[3]),
    servo.Servo(pca.channels[4]),
    servo.Servo(pca.channels[5]),
]

print("Moving all servos to 90 degrees...")

for i, s in enumerate(servos):
    try:
        s.angle = 90
    except ValueError:
        print(f"Servo {i} reported a pulse width error.")

print("Done. Servos should be holding position at 90.")