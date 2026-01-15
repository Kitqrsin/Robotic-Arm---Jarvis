from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50


# Set duty cycle to 0 for all 16 channels to kill the signal completely
def stop_all_servos():
    print("Resetting all channels...")
    for i in range(16):
        pca.channels[i].duty_cycle = 0

    print("All servos should now be limp/stopped.")


if __name__ == "__main__":
    stop_all_servos()
