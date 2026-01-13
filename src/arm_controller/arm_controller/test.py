import time
import sys
import os
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import OutputDevice  # For OE Pin control

# Add parent directory to path to import arm_poses
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))
# Try/Except block in case arm_poses file is missing for testing
try:
    from arm_poses import POSES
except ImportError:
    POSES = {}
    print("Warning: arm_poses.py not found. Pose commands will be disabled.")

# --- CONFIGURATION SECTION ---
# Define the Output Enable (OE) Pin. GPIO 17 is standard, but check your wiring!
OE_PIN = 17 

SERVO_CONFIG = {
    # BASE: Continuous Rotation (360). 
    # 'base': {'channel': 0, 'type': 'continuous', 'min': 500, 'max': 2500}, 
    
    # ARM JOINTS: Standard Positional Servos (Angles).
    's': {'channel': 1, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 90},
    'e': {'channel': 2, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 90},
    'f': {'channel': 4, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 80},
    'w': {'channel': 5, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 60},
    'g': {'channel': 6, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 45},
}

# --- 1. IMMEDIATE SAFETY SETUP ---
# We must disable motors BEFORE initializing the I2C bus to prevent slamming.
try:
    oe_pin = OutputDevice(OE_PIN, initial_value=True)  # HIGH = MOTORS DISABLED
    print(f"\n[SAFETY] OE Pin {OE_PIN} is HIGH. Motors are PHYSICALLY DISABLED.")
except Exception as e:
    print(f"[ERROR] Could not setup GPIO: {e}")
    print("WARNING: Running without OE pin safety!")
    oe_pin = None

# --- INITIALIZATION ---
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

arm = {}

def stop_all_servos():
    """Stops all motors safely and DISABLES power via OE pin."""
    print("\n! STOPPING ALL SERVOS !")
    
    # 1. Soft Stop (Logic)
    if 'base' in arm:
        arm['base'].throttle = 0
    
    # 2. Cut PWM Signal
    for i in range(16):
        pca.channels[i].duty_cycle = 0

    # 3. Hard Stop (Cut Power via OE)
    try:
        if oe_pin:
            oe_pin.on()  # HIGH = Disable motors
            print("[SAFETY] Motors Disabled.")
    except Exception as e:
        print(f"[WARNING] Could not disable OE pin: {e}")

print("--- INITIALIZING ROBOT LOGIC ---")

# We initialize the software objects, but the physical motors are still OFF.
for name, config in SERVO_CONFIG.items():
    try:
        if config['type'] == 'continuous':
            s = servo.ContinuousServo(
                pca.channels[config['channel']], 
                min_pulse=config['min'], 
                max_pulse=config['max']
            )
            s.throttle = 0 
            print(f"  > {name.capitalize()} (360°) logic ready.")
            
        else:
            s = servo.Servo(
                pca.channels[config['channel']], 
                min_pulse=config['min'], 
                max_pulse=config['max']
            )
            s.actuation_range = config['range']
            # Store servo object and start angle (don't write yet)
            arm[name] = s
            if config['start_angle'] is not None:
                print(f"  > {name.capitalize()} will start at {config['start_angle']}°")
        
        if config['type'] == 'continuous':
            arm[name] = s
        
    except Exception as e:
        print(f"  ! Error initializing {name}: {e}")

# --- 2. USER TRIGGER TO START ---
print("\n" + "="*40)
print(" SYSTEM READY (Motors currently DISABLED)")
print(" Checks completed. The motors have not moved.")
print("="*40)

input("👉 Press [ENTER] to ENABLE motors and Start...")

# --- 3. ENABLE MOTORS ---
print("Enabling in 3...", end="", flush=True)
time.sleep(0.5)
print(" 2...", end="", flush=True)
time.sleep(0.5)
print(" 1...", flush=True)

try:
    if oe_pin:
        oe_pin.off()  # LOW = MOTORS ENABLED
        print("⚡ MOTORS ACTIVE ⚡")
    else:
        print("⚡ Starting without OE control ⚡")
except Exception as e:
    print(f"[ERROR] Could not enable motors: {e}")
    print("Continuing without OE pin control...")
time.sleep(0.5) # Allow voltage to stabilize

# Now move servos to start positions
print("Moving to start positions...")
for name, config in SERVO_CONFIG.items():
    if config['type'] == 'standard' and config['start_angle'] is not None:
        if name in arm:
            arm[name].angle = config['start_angle']
            print(f"  > {name.capitalize()} locked at {config['start_angle']}°")
print("--- READY ---")


# --- HELPER FUNCTIONS ---

def move_base(throttle, duration):
    if 'base' not in arm:
        print("Base not configured.")
        return
    print(f"Spinning Base at speed {throttle} for {duration} seconds...")
    arm['base'].throttle = throttle
    time.sleep(duration)
    arm['base'].throttle = 0

def move_smooth(joint_name, target_angle, delay=0.08):
    if joint_name == 'base': return
    if joint_name not in arm: return

    s = arm[joint_name]
    
    # Slower movement for shoulder/elbow
    if joint_name in ['s', 'e']:
        delay = 0.12 
    
    current_angle = s.angle if s.angle is not None else target_angle
    step = 1 if target_angle > current_angle else -1
    
    if int(current_angle) != int(target_angle):
        for angle in range(int(current_angle), int(target_angle), step):
            s.angle = angle
            time.sleep(delay)
    
    s.angle = target_angle

def execute_pose(pose_name):
    if pose_name not in POSES:
        print(f"Pose '{pose_name}' not found!")
        return
    
    pose = POSES[pose_name]
    print(f"\n🎯 Executing pose: {pose_name.upper()}")
    
    joint_mapping = {'shoulder': 's', 'elbow': 'e', 'forearm': 'f', 'wrist': 'w', 'gripper': 'g'}
    
    for joint_name, angle in pose.items():
        if joint_name == 'description': continue
        
        servo_name = joint_mapping.get(joint_name)
        if servo_name and servo_name in arm:
            print(f"   → {joint_name} {angle}")
            move_smooth(servo_name, angle)
            time.sleep(0.2)
            
    if pose_name == 'wave':
        print("\n   🌊 Waving...")
        for i in range(3):
            move_smooth('f', 70, 0.1)
            time.sleep(0.4)
            move_smooth('f', 120, 0.1)
            time.sleep(0.4)
        move_smooth('f', 50)
    
    print(f"✓ Pose '{pose_name}' complete\n")

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    try:
        print("\nCOMMAND GUIDE:")
        print("1. Joints: 's 90', 'e 120', 'g 160'")
        print("2. Poses:  'home', 'wave', etc.")
        print("Type 'q' to quit.")
        
        while True:
            user_input = input("Command: ").lower().split()
            if not user_input: continue
                
            cmd = user_input[0]
            
            if cmd == 'q':
                stop_all_servos()
                try:
                    if oe_pin:
                        oe_pin.close()
                    print("[CLEANUP] GPIO pins released.")
                except Exception:
                    pass
                break
            
            elif cmd in POSES:
                execute_pose(cmd)
            
            elif cmd == 'base' and len(user_input) == 3:
                try:
                    move_base(float(user_input[1]), float(user_input[2]))
                except ValueError: pass

            elif len(user_input) == 2:
                try:
                    move_smooth(cmd, int(user_input[1]))
                except ValueError: pass
            else:
                print("Unknown command.")
                
    except KeyboardInterrupt:
        stop_all_servos()
        try:
            if oe_pin:
                oe_pin.close()
            print("\n[CLEANUP] GPIO pins released.")
        except Exception:
            pass
        print("Program stopped.")