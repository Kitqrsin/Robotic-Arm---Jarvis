import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- CONFIGURATION SECTION ---
# 'start_angle': The safe position the robot should snap to immediately on boot.
# 'min'/'max': Your specific calibration values.
SERVO_CONFIG = {
    # 'base':     {'channel': 0, 'min': 500, 'max': 2500, 'range': 360, 'start_angle': 90}, 
    'shoulder': {'channel': 1, 'min': 500, 'max': 2500, 'range': 180, 'start_angle': 90},  # Starts folded down
    'elbow':    {'channel': 2, 'min': 500, 'max': 2500, 'range': 180, 'start_angle': 90},  # Starts folded in
    'wrist_p':  {'channel': 4, 'min': 500, 'max': 2500, 'range': 180, 'start_angle': 90},
    'wrist_r':  {'channel': 5, 'min': 500, 'max': 2500, 'range': 180, 'start_angle': 90},
    'gripper':  {'channel': 6, 'min': 500, 'max': 2500, 'range': 180, 'start_angle': 0},
}

# --- INITIALIZATION ---
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Dictionary to hold the servo objects
arm = {}

def stop_all_servos():
    """Turns off the signal to all motors (makes them go limp)."""
    print("\n! CUTTING POWER TO SERVOS !")
    for i in range(16):
        pca.channels[i].duty_cycle = 0

print("--- INITIALIZING & ENGAGING MOTORS ---")
for name, config in SERVO_CONFIG.items():
    try:
        # 1. Create the servo object
        s = servo.Servo(
            pca.channels[config['channel']], 
            min_pulse=config['min'], 
            max_pulse=config['max']
        )
        s.actuation_range = config['range']
        
        # 2. SOFT START: Immediately lock to the start angle
        # This prevents the motor from being "limp" until you send a command
        if config['start_angle'] is not None:
            s.angle = config['start_angle']
        
        arm[name] = s
        print(f"  > {name.capitalize()} locked at {config['start_angle']}°")
        time.sleep(0.1) # Small delay to prevent power spike
        
    except Exception as e:
        print(f"  ! Error initializing {name}: {e}")

print("--- MOTORS ARE ACTIVE AND HOLDING ---")

# --- HELPER FUNCTIONS ---

def move_smooth(joint_name, target_angle, delay=0.01):
    """
    Moves a joint smoothly.
    Keeps the signal active at the end to HOLD position.
    """
    # Handle moving ALL motors at once (Good for testing holding power)
    if joint_name == 'all':
        print(f"Moving ALL servos to {target_angle}...")
        for name, servo_obj in arm.items():
            servo_obj.angle = target_angle
        return

    # Check if joint exists
    if joint_name not in arm:
        print(f"Error: '{joint_name}' is not a valid joint name.")
        return

    s = arm[joint_name]
    
    # Get current angle (safely handle None)
    start_angle = s.angle if s.angle is not None else target_angle
    
    # Determine step direction (up or down)
    step = 1 if target_angle > start_angle else -1
    
    # Move gradually
    if int(start_angle) != int(target_angle):
        for angle in range(int(start_angle), int(target_angle), step):
            s.angle = angle
            time.sleep(delay) 
    
    # FORCE HOLD: Explicitly set the final angle again to ensure it locks
    s.angle = target_angle
    # We do NOT turn off the duty cycle here. This keeps the motor straining/holding.

def demo_sequence():
    print("\n--- STARTING DEMO ---")
    
    # 1. Home Position
    print("Homing...")
    if 'base' in arm: move_smooth('base', 0)
    move_smooth('shoulder', 90)
    move_smooth('elbow', 90)
    move_smooth('wrist_p', 90)
    move_smooth('wrist_r', 90)
    move_smooth('gripper', 0)
    time.sleep(1)

    # 2. Grab Action
    print("Grabbing...")
    move_smooth('shoulder', 45)
    move_smooth('elbow', 45)
    time.sleep(0.5)
    
    move_smooth('gripper', 120) 
    time.sleep(0.5)

    # 3. Return
    print("Retracting...")
    move_smooth('shoulder', 90)
    move_smooth('elbow', 90)
    
    # 4. Rotate Base (Only if base exists)
    if 'base' in arm:
        print("Rotating...")
        move_smooth('base', 180)
    
    print("--- DEMO COMPLETE ---")

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    try:
        print("\nInteractive Mode: Type 'joint angle' (e.g., 'shoulder 90')")
        print("Type 'all 90' to move everything.")
        print("Type 'demo' to run the sequence.")
        print("Type 'q' to quit (WARNING: Motors will go limp).")
        
        while True:
            user_input = input("Command: ").lower().split()
            
            # Handle empty input
            if not user_input:
                continue
                
            cmd = user_input[0]
            
            if cmd == 'q':
                stop_all_servos()
                break
            
            elif cmd == 'demo':
                demo_sequence()
                
            elif len(user_input) == 2:
                try:
                    name = cmd
                    angle = int(user_input[1])
                    move_smooth(name, angle)
                except ValueError:
                    print("Error: Angle must be a number.")
            else:
                print("Invalid format. Use: name angle")
                
    except KeyboardInterrupt:
        # This catches Ctrl+C
        stop_all_servos()
        print("\nProgram stopped.")