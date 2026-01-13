import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- CONFIGURATION SECTION ---
SERVO_CONFIG = {
    # BASE: Continuous Rotation (360). 
    # 'type': 'continuous' tells the script to use speed/time instead of angles.
    # 'base': {'channel': 0, 'type': 'continuous', 'min': 500, 'max': 2500}, 
    
    # ARM JOINTS: Standard Positional Servos (Angles).
    # 'start_angle': The safe angle to lock to on boot.
    's': {'channel': 1, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 108},
    'e':    {'channel': 2, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 98},
    'f':  {'channel': 4, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 80},
    'w':    {'channel': 5, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 60},
    'g':  {'channel': 6, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 45},
}

# --- INITIALIZATION ---
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

arm = {}

def stop_all_servos():
    """Stops all motors safely."""
    print("\n! STOPPING ALL SERVOS !")
    # For continuous servos, we must set throttle to 0
    if 'base' in arm:
        arm['base'].throttle = 0
    
    # Finally cut the signal completely
    for i in range(16):
        pca.channels[i].duty_cycle = 0

print("--- INITIALIZING ROBOT ---")

for name, config in SERVO_CONFIG.items():
    try:
        if config['type'] == 'continuous':
            # Initialize as Continuous Servo (Speed based)
            s = servo.ContinuousServo(
                pca.channels[config['channel']], 
                min_pulse=config['min'], 
                max_pulse=config['max']
            )
            s.throttle = 0 # Ensure it is STOPPED on startup
            print(f"  > {name.capitalize()} (360°) initialized. Holding STOP.")
            
        else:
            # Initialize as Standard Servo (Angle based)
            s = servo.Servo(
                pca.channels[config['channel']], 
                min_pulse=config['min'], 
                max_pulse=config['max']
            )
            s.actuation_range = config['range']
            # Soft Start
            if config['start_angle'] is not None:
                s.angle = config['start_angle']
                print(f"  > {name.capitalize()} locked at {config['start_angle']}°")
        
        arm[name] = s
        time.sleep(0.1)
        
    except Exception as e:
        print(f"  ! Error initializing {name}: {e}")

print("--- READY ---")

# --- HELPER FUNCTIONS ---

def move_base(throttle, duration):
    """
    Moves the continuous rotation base.
    throttle: -1.0 (Full Back) to 1.0 (Full Fwd). 0 is Stop.
    duration: Seconds to spin before stopping.
    """
    if 'base' not in arm:
        print("Base not configured.")
        return

    print(f"Spinning Base at speed {throttle} for {duration} seconds...")
    arm['base'].throttle = throttle
    time.sleep(duration)
    arm['base'].throttle = 0
    print("Base Stopped.")

def move_smooth(joint_name, target_angle, delay=0.01):
    """Moves standard positional servos smoothly."""
    if joint_name == 'base':
        print("Error: Use 'base [speed] [seconds]' for the base motor.")
        return

    if joint_name not in arm:
        print(f"Error: '{joint_name}' not found.")
        return

    s = arm[joint_name]
    
    # Safety check for None (if motor glitch occurred)
    current_angle = s.angle if s.angle is not None else target_angle
    
    step = 1 if target_angle > current_angle else -1
    
    if int(current_angle) != int(target_angle):
        for angle in range(int(current_angle), int(target_angle), step):
            s.angle = angle
            time.sleep(delay)
    
    s.angle = target_angle

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    try:
        print("\nCOMMAND GUIDE:")
        print("1. Standard Joints: 'shoulder 90', 'gripper 180'")
        print("2. Base (360):      'base 0.5 2'  (Spin at 50% speed for 2 seconds)")
        print("                    'base -0.5 1' (Spin reverse 50% speed for 1 second)")
        print("Type 'q' to quit.")
        
        while True:
            user_input = input("Command: ").lower().split()
            if not user_input: continue
                
            cmd = user_input[0]
            
            if cmd == 'q':
                stop_all_servos()
                break
            
            # --- BASE COMMAND LOGIC ---
            elif cmd == 'base':
                if len(user_input) == 3:
                    try:
                        speed = float(user_input[1])
                        seconds = float(user_input[2])
                        # Safety limits for throttle
                        if speed > 1.0: speed = 1.0
                        if speed < -1.0: speed = -1.0
                        move_base(speed, seconds)
                    except ValueError:
                        print("Error: Use numbers. Example: base 0.5 2")
                else:
                    print("Invalid Base Format. Use: base [speed -1.0 to 1.0] [seconds]")

            # --- STANDARD ARM LOGIC ---
            elif len(user_input) == 2:
                try:
                    name = cmd
                    angle = int(user_input[1])
                    move_smooth(name, angle)
                except ValueError:
                    print("Error: Angle must be an integer.")
            else:
                print("Unknown command.")
                
    except KeyboardInterrupt:
        stop_all_servos()
        print("\nProgram stopped.")