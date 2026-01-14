#!/usr/bin/env python3
"""
Flask Web Interface for Robot Arm Control
Provides a web-based UI to control servos and save/load poses
"""
from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
import time
import sys
import os
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from gpiozero import OutputDevice
import threading

# Add parent directory to path to import arm_poses
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))
try:
    from arm_poses import POSES
except ImportError:
    POSES = {}

app = Flask(__name__)
CORS(app)

# --- CONFIGURATION ---
OE_PIN = 17

SERVO_CONFIG = {
    's': {'channel': 1, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 90},
    'e': {'channel': 2, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 90},
    'f': {'channel': 4, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 80},
    'w': {'channel': 5, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 60},
    'g': {'channel': 6, 'type': 'standard', 'min': 1000, 'max': 2000, 'range': 180, 'start_angle': 45},
}

# Mapping from web UI servo numbers to internal names
SERVO_MAPPING = {
    1: 's',   # Base (not configured yet, but reserved)
    2: 's',   # Shoulder
    3: 'e',   # Elbow
    4: 'f',   # Forearm
    5: 'w',   # Wrist
    6: 'g'    # Gripper
}

# --- HARDWARE INITIALIZATION ---
oe_pin = None
pca = None
arm = {}
arm_lock = threading.Lock()

def init_hardware():
    """Initialize GPIO, I2C, and servo objects"""
    global oe_pin, pca, arm
    
    # Setup OE Pin
    try:
        oe_pin = OutputDevice(OE_PIN, initial_value=True)  # HIGH = MOTORS DISABLED
        print(f"[SAFETY] OE Pin {OE_PIN} is HIGH. Motors are PHYSICALLY DISABLED.")
    except Exception as e:
        print(f"[ERROR] Could not setup GPIO: {e}")
        print("WARNING: Running without OE pin safety!")
        oe_pin = None

    # Initialize I2C and PCA9685
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Create servo objects
    print("--- INITIALIZING SERVOS ---")
    for name, config in SERVO_CONFIG.items():
        try:
            s = servo.Servo(
                pca.channels[config['channel']], 
                min_pulse=config['min'], 
                max_pulse=config['max']
            )
            s.actuation_range = config['range']
            arm[name] = s
            print(f"  > {name.upper()} initialized on channel {config['channel']}")
        except Exception as e:
            print(f"  ! Error initializing {name}: {e}")

    # Enable motors
    if oe_pin:
        oe_pin.off()  # LOW = MOTORS ENABLED
        print("⚡ MOTORS ACTIVE ⚡")
        time.sleep(0.5)

    # Move to start positions
    print("Moving to start positions...")
    for name, config in SERVO_CONFIG.items():
        if config['start_angle'] is not None and name in arm:
            arm[name].angle = config['start_angle']
            print(f"  > {name.upper()} locked at {config['start_angle']}°")

    print("--- READY ---")

# --- API ROUTES ---

@app.route('/')
def index():
    """Serve the main HTML UI"""
    return render_template('index.html')

@app.route('/api/servo/<int:servo_id>', methods=['POST'])
def set_servo(servo_id):
    """Set a single servo position"""
    try:
        data = request.get_json()
        angle = int(data.get('angle', 90))
        
        # Clamp angle
        angle = max(0, min(180, angle))
        
        # Map web UI servo ID to internal servo name
        servo_name = SERVO_MAPPING.get(servo_id)
        
        if not servo_name or servo_name not in arm:
            return jsonify({'success': False, 'error': f'Invalid servo ID: {servo_id}'}), 400
        
        with arm_lock:
            arm[servo_name].angle = angle
        
        return jsonify({'success': True, 'servo': servo_id, 'angle': angle})
    
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/servos/batch', methods=['POST'])
def set_servos_batch():
    """Set multiple servos at once"""
    try:
        data = request.get_json()
        servos = data.get('servos', {})  # {1: 90, 2: 120, ...}
        
        results = []
        with arm_lock:
            for servo_id, angle in servos.items():
                servo_id = int(servo_id)
                angle = max(0, min(180, int(angle)))
                
                servo_name = SERVO_MAPPING.get(servo_id)
                if servo_name and servo_name in arm:
                    arm[servo_name].angle = angle
                    results.append({'servo': servo_id, 'angle': angle, 'success': True})
                else:
                    results.append({'servo': servo_id, 'success': False, 'error': 'Invalid servo'})
        
        return jsonify({'success': True, 'results': results})
    
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/emergency_stop', methods=['POST'])
def emergency_stop():
    """Emergency stop - disable all servos"""
    try:
        with arm_lock:
            # Cut PWM signals
            for i in range(16):
                pca.channels[i].duty_cycle = 0
            
            # Disable motors via OE pin
            if oe_pin:
                oe_pin.on()  # HIGH = MOTORS DISABLED
        
        return jsonify({'success': True, 'message': 'Emergency stop activated'})
    
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/poses', methods=['GET'])
def get_poses():
    """Get all predefined poses"""
    pose_list = []
    for name, pose_data in POSES.items():
        pose_list.append({
            'name': name,
            'description': pose_data.get('description', ''),
            'angles': {k: v for k, v in pose_data.items() if k != 'description'}
        })
    return jsonify({'success': True, 'poses': pose_list})

@app.route('/api/status', methods=['GET'])
def get_status():
    """Get current arm status"""
    status = {
        'online': True,
        'oe_enabled': oe_pin is not None and not oe_pin.value,
        'servos': {}
    }
    
    with arm_lock:
        for name, servo_obj in arm.items():
            try:
                status['servos'][name] = {
                    'angle': servo_obj.angle,
                    'channel': SERVO_CONFIG[name]['channel']
                }
            except:
                status['servos'][name] = {'angle': None, 'channel': SERVO_CONFIG[name]['channel']}
    
    return jsonify(status)

# --- CLEANUP ---
def cleanup():
    """Cleanup GPIO and servos on shutdown"""
    print("\n[SHUTDOWN] Cleaning up...")
    try:
        for i in range(16):
            pca.channels[i].duty_cycle = 0
        if oe_pin:
            oe_pin.on()  # Disable motors
            oe_pin.close()
        print("[CLEANUP] Done.")
    except Exception as e:
        print(f"[CLEANUP ERROR] {e}")

# --- MAIN ---
if __name__ == '__main__':
    try:
        init_hardware()
        print("\n" + "="*50)
        print("  🌐 Web UI available at http://localhost:5000")
        print("="*50 + "\n")
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        cleanup()
    except Exception as e:
        print(f"[ERROR] {e}")
        cleanup()
