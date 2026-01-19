#!/usr/bin/env python3
"""
Flask Web Interface for Robot Arm Control
Provides a web-based UI to control servos and save/load poses
"""

import atexit
import json
import os
import signal
import sqlite3
import sys
import threading
import time
from functools import wraps

import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
from flask import (
    Flask,
    flash,
    jsonify,
    redirect,
    render_template,
    request,
    session,
    url_for,
)
from flask_cors import CORS
from gpiozero import OutputDevice
from werkzeug.security import check_password_hash, generate_password_hash

# Add parent directory to path to import arm_poses
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))
try:
    from arm_poses import POSES
except ImportError:
    POSES = {}

app = Flask(__name__)
app.secret_key = (
    os.environ.get("SECRET_KEY")
    or "dev-secret-key-change-in-production-" + os.urandom(24).hex()
)
CORS(app)

# --- CONFIGURATION ---
OE_PIN = 17
DB_PATH = os.path.join(os.path.dirname(__file__), "robot_poses.db")
STATE_FILE = os.path.join(os.path.dirname(__file__), "last_position.json")

SERVO_CONFIG = {
    "b": {
        "channel": 0,
        "type": "standard",
        "min": 1000,
        "max": 2000,
        "range": 270,
        "start_angle": 98,
    },
    "s": {
        "channel": 1,
        "type": "standard",
        "min": 1000,
        "max": 2000,
        "range": 180,
        "start_angle": 96,
    },
    "e": {
        "channel": 2,
        "type": "standard",
        "min": 1000,
        "max": 2000,
        "range": 180,
        "start_angle": 98,
    },
    "f": {
        "channel": 4,
        "type": "standard",
        "min": 1000,
        "max": 2000,
        "range": 180,
        "start_angle": 70,
    },
    "w": {
        "channel": 5,
        "type": "standard",
        "min": 1000,
        "max": 2000,
        "range": 180,
        "start_angle": 60,
    },
    "g": {
        "channel": 6,
        "type": "standard",
        "min": 500,
        "max": 2500,
        "range": 180,
        "start_angle": 45,
    },
}

# Mapping from web UI servo numbers to internal names
SERVO_MAPPING = {
    1: "b",  # Base
    2: "s",  # Shoulder
    3: "e",  # Elbow
    4: "f",  # Forearm
    5: "w",  # Wrist
    6: "g",  # Gripper
}

# --- HARDWARE INITIALIZATION ---
oe_pin = None
pca = None
arm = {}
arm_lock = threading.Lock()
current_position = {}  # Track current servo positions

# --- POSITION PERSISTENCE FUNCTIONS ---


def save_current_position():
    """Save current arm position to file"""
    try:
        with arm_lock:
            position = {
                "s1": current_position.get("b", SERVO_CONFIG["b"]["start_angle"]),
                "s2": current_position.get("s", SERVO_CONFIG["s"]["start_angle"]),
                "s3": current_position.get("e", SERVO_CONFIG["e"]["start_angle"]),
                "s4": current_position.get("f", SERVO_CONFIG["f"]["start_angle"]),
                "s5": current_position.get("w", SERVO_CONFIG["w"]["start_angle"]),
                "s6": current_position.get("g", SERVO_CONFIG["g"]["start_angle"]),
            }

        with open(STATE_FILE, "w") as f:
            json.dump(position, f, indent=2)

        print(f"[STATE] Saved current position: {position}")
        return True
    except Exception as e:
        print(f"[ERROR] Could not save position: {e}")
        return False


def load_last_position():
    """Load last saved position from file"""
    try:
        if os.path.exists(STATE_FILE):
            with open(STATE_FILE, "r") as f:
                position = json.load(f)
            print(f"[STATE] Loaded last position: {position}")
            return position
        else:
            print("[STATE] No saved position found, using defaults")
            return None
    except Exception as e:
        print(f"[ERROR] Could not load position: {e}")
        return None


# --- DATABASE FUNCTIONS ---


def init_database():
    """Initialize SQLite database for saved poses and users"""
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # Poses table
    cursor.execute(
        """
        CREATE TABLE IF NOT EXISTS poses (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            s1 INTEGER NOT NULL,
            s2 INTEGER NOT NULL,
            s3 INTEGER NOT NULL,
            s4 INTEGER NOT NULL,
            s5 INTEGER NOT NULL,
            s6 INTEGER NOT NULL,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """
    )

    # Users table
    cursor.execute(
        """
        CREATE TABLE IF NOT EXISTS users (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            username TEXT UNIQUE NOT NULL,
            password_hash TEXT NOT NULL,
            is_admin BOOLEAN DEFAULT 0,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """
    )

    # Migration: Add is_admin column if it doesn't exist
    try:
        cursor.execute("SELECT is_admin FROM users LIMIT 1")
    except sqlite3.OperationalError:
        print("[DATABASE] Migrating: Adding is_admin column to users table")
        cursor.execute("ALTER TABLE users ADD COLUMN is_admin BOOLEAN DEFAULT 0")
        conn.commit()
        print("[DATABASE] Migration complete")

    # Migration: Add s1 column to poses table if it doesn't exist
    try:
        cursor.execute("SELECT s1 FROM poses LIMIT 1")
    except sqlite3.OperationalError:
        print("[DATABASE] Migrating: Adding s1 column to poses table")
        cursor.execute("ALTER TABLE poses ADD COLUMN s1 INTEGER DEFAULT 135")
        conn.commit()
        print("[DATABASE] Migration complete")

    conn.commit()
    conn.close()
    print("[DATABASE] Initialized robot_poses.db with users table")


def init_admin():
    """Create or update default admin account"""
    conn = get_db_connection()
    cursor = conn.cursor()

    # Check if admin exists
    cursor.execute("SELECT id FROM users WHERE username = ?", ("admin",))
    existing_admin = cursor.fetchone()
    
    admin_password_hash = generate_password_hash("256admin")
    
    if not existing_admin:
        # Create admin account
        cursor.execute(
            "INSERT INTO users (username, password_hash, is_admin) VALUES (?, ?, ?)",
            ("admin", admin_password_hash, 1),
        )
        print("[ADMIN] Created default admin account (username: admin)")
    else:
        # Update existing admin password
        cursor.execute(
            "UPDATE users SET password_hash = ? WHERE username = ?",
            (admin_password_hash, "admin")
        )
        print("[ADMIN] Updated admin password")

    conn.commit()
    conn.close()


def get_db_connection():
    """Get a database connection"""
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


def login_required(f):
    """Decorator to require login for routes"""

    @wraps(f)
    def decorated_function(*args, **kwargs):
        if "user_id" not in session:
            if request.path.startswith("/api/"):
                return (
                    jsonify({"success": False, "error": "Authentication required"}),
                    401,
                )
            return redirect(url_for("login"))
        return f(*args, **kwargs)

    return decorated_function


def admin_required(f):
    """Decorator to require admin privileges"""

    @wraps(f)
    def decorated_function(*args, **kwargs):
        if "user_id" not in session:
            return redirect(url_for("login"))
        if not session.get("is_admin", False):
            flash("Admin privileges required", "error")
            return redirect(url_for("index"))
        return f(*args, **kwargs)

    return decorated_function


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
                pca.channels[config["channel"]],
                min_pulse=config["min"],
                max_pulse=config["max"],
            )
            s.actuation_range = config["range"]
            arm[name] = s
            print(f"  > {name.upper()} initialized on channel {config['channel']}")
        except Exception as e:
            print(f"  ! Error initializing {name}: {e}")

    # Keep motors disabled on startup
    print("--- MOTORS DISABLED (OE Pin HIGH) ---")
    print("--- Use the web UI to enable motors ---")
    print("--- READY ---")


# --- AUTHENTICATION ROUTES ---


@app.route("/login", methods=["GET", "POST"])
def login():
    """Login page and handler"""
    if request.method == "POST":
        username = request.form.get("username", "").strip()
        password = request.form.get("password", "")

        if not username or not password:
            flash("Username and password are required", "error")
            return render_template("login.html")

        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM users WHERE username = ?", (username,))
        user = cursor.fetchone()
        conn.close()

        if user and check_password_hash(user["password_hash"], password):
            session["user_id"] = user["id"]
            session["username"] = user["username"]
            session["is_admin"] = bool(user["is_admin"])
            flash("Login successful!", "success")
            return redirect(url_for("index"))
        else:
            flash("Invalid username or password", "error")

    return render_template("login.html")


@app.route("/register", methods=["GET", "POST"])
@admin_required
def register():
    """Registration page and handler - Admin only"""
    if request.method == "POST":
        username = request.form.get("username", "").strip()
        password = request.form.get("password", "")
        confirm_password = request.form.get("confirm_password", "")

        if not username or not password:
            flash("Username and password are required", "error")
            return render_template("register.html")

        if len(username) < 3:
            flash("Username must be at least 3 characters", "error")
            return render_template("register.html")

        if len(password) < 6:
            flash("Password must be at least 6 characters", "error")
            return render_template("register.html")

        if password != confirm_password:
            flash("Passwords do not match", "error")
            return render_template("register.html")

        conn = get_db_connection()
        cursor = conn.cursor()

        # Check if username exists
        cursor.execute("SELECT id FROM users WHERE username = ?", (username,))
        if cursor.fetchone():
            conn.close()
            flash("Username already exists", "error")
            return render_template("register.html")

        # Create user
        password_hash = generate_password_hash(password)
        cursor.execute(
            "INSERT INTO users (username, password_hash, is_admin) VALUES (?, ?, ?)",
            (username, password_hash, 0),
        )
        conn.commit()
        conn.close()

        flash(f'User "{username}" created successfully!', "success")
        return redirect(url_for("register"))

    return render_template("register.html")


@app.route("/logout")
def logout():
    """Logout handler"""
    session.clear()
    flash("Logged out successfully", "success")
    return redirect(url_for("login"))


# --- API ROUTES ---


@app.route("/")
@login_required
def index():
    """Serve the main HTML UI"""
    return render_template("index.html", username=session.get("username"))


@app.route("/api/servo/<int:servo_id>", methods=["POST"])
@login_required
def set_servo(servo_id):
    """Set a single servo position"""
    try:
        data = request.get_json()
        angle = int(data.get("angle", 90))

        # Clamp angle
        angle = max(0, min(180, angle))

        # Map web UI servo ID to internal servo name
        servo_name = SERVO_MAPPING.get(servo_id)

        if not servo_name or servo_name not in arm:
            return (
                jsonify({"success": False, "error": f"Invalid servo ID: {servo_id}"}),
                400,
            )

        with arm_lock:
            arm[servo_name].angle = angle
            current_position[servo_name] = angle  # Track position

        return jsonify({"success": True, "servo": servo_id, "angle": angle})

    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/api/servos/batch", methods=["POST"])
@login_required
def set_servos_batch():
    """Set multiple servos at once"""
    try:
        data = request.get_json()
        servos = data.get("servos", {})  # {1: 90, 2: 120, ...}

        results = []
        with arm_lock:
            for servo_id, angle in servos.items():
                servo_id = int(servo_id)
                angle = max(0, min(180, int(angle)))

                servo_name = SERVO_MAPPING.get(servo_id)
                if servo_name and servo_name in arm:
                    arm[servo_name].angle = angle
                    current_position[servo_name] = angle  # Track position
                    results.append({"servo": servo_id, "angle": angle, "success": True})
                else:
                    results.append(
                        {"servo": servo_id, "success": False, "error": "Invalid servo"}
                    )

        return jsonify({"success": True, "results": results})

    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/api/emergency_stop", methods=["POST"])
@login_required
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

        return jsonify(
            {
                "success": True,
                "message": "Emergency stop activated",
                "oe_enabled": False,
            }
        )

    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/api/oe_toggle", methods=["POST"])
@login_required
def oe_toggle():
    """Toggle OE pin (enable/disable motors)"""
    try:
        data = request.get_json()
        enable = data.get("enable", False)  # True = enable motors, False = disable

        if not oe_pin:
            return jsonify({"success": False, "error": "OE pin not available"}), 400

        if enable:
            oe_pin.off()  # LOW = MOTORS ENABLED
            time.sleep(0.5)
            # Don't move servos here - let the frontend handle gradual movement
            message = "Motors ENABLED"
        else:
            with arm_lock:
                # Cut PWM first
                for i in range(16):
                    pca.channels[i].duty_cycle = 0
            oe_pin.on()  # HIGH = MOTORS DISABLED
            message = "Motors DISABLED"

        return jsonify({"success": True, "message": message, "oe_enabled": enable})

    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/api/saved_poses", methods=["GET"])
@login_required
def get_saved_poses():
    """Get all user-saved poses from database"""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM poses ORDER BY created_at DESC")
        rows = cursor.fetchall()
        conn.close()

        poses = []
        for row in rows:
            poses.append(
                {
                    "id": row["id"],
                    "name": row["name"],
                    "state": {
                        "s1": row["s1"],
                        "s2": row["s2"],
                        "s3": row["s3"],
                        "s4": row["s4"],
                        "s5": row["s5"],
                        "s6": row["s6"],
                    },
                    "timestamp": row["created_at"],
                }
            )

        return jsonify({"success": True, "poses": poses})

    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/api/saved_poses", methods=["POST"])
@login_required
def save_pose():
    """Save a new pose to database"""
    try:
        data = request.get_json()
        name = data.get("name", "").strip()
        state = data.get("state", {})

        if not name:
            return jsonify({"success": False, "error": "Name is required"}), 400

        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute(
            """
            INSERT INTO poses (name, s1, s2, s3, s4, s5, s6)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """,
            (
                name,
                state.get("s1", 90),
                state.get("s2", 90),
                state.get("s3", 90),
                state.get("s4", 80),
                state.get("s5", 60),
                state.get("s6", 45),
            ),
        )

        pose_id = cursor.lastrowid
        conn.commit()
        conn.close()

        return jsonify(
            {"success": True, "id": pose_id, "message": f'Pose "{name}" saved'}
        )

    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/api/saved_poses/<int:pose_id>", methods=["DELETE"])
@login_required
def delete_pose(pose_id):
    """Delete a saved pose"""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute("DELETE FROM poses WHERE id = ?", (pose_id,))
        conn.commit()
        conn.close()

        return jsonify({"success": True, "message": "Pose deleted"})

    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/api/poses", methods=["GET"])
@login_required
def get_poses():
    """Get all predefined poses"""
    pose_list = []
    for name, pose_data in POSES.items():
        pose_list.append(
            {
                "name": name,
                "description": pose_data.get("description", ""),
                "angles": {k: v for k, v in pose_data.items() if k != "description"},
            }
        )
    return jsonify({"success": True, "poses": pose_list})


@app.route("/api/status", methods=["GET"])
@login_required
def get_status():
    """Get current arm status"""
    status = {
        "online": True,
        "oe_enabled": oe_pin is not None
        and not oe_pin.is_active,  # is_active means LOW (enabled)
        "servos": {},
    }

    with arm_lock:
        for name, servo_obj in arm.items():
            try:
                status["servos"][name] = {
                    "angle": servo_obj.angle,
                    "channel": SERVO_CONFIG[name]["channel"],
                }
            except Exception:
                status["servos"][name] = {
                    "angle": None,
                    "channel": SERVO_CONFIG[name]["channel"],
                }

    return jsonify(status)


@app.route("/api/last_position", methods=["GET"])
@login_required
def get_last_position():
    """Get last saved position"""
    position = load_last_position()
    if position:
        return jsonify({"success": True, "position": position})
    else:
        return jsonify({"success": False, "message": "No saved position"})


@app.route("/api/initial_positions", methods=["GET"])
@login_required
def get_initial_positions():
    """Get initial positions from database 'reset' pose or fallback to SERVO_CONFIG"""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM poses WHERE name = ?", ("reset",))
        reset_pose = cursor.fetchone()
        conn.close()

        if reset_pose:
            # Use database values
            initial_positions = {
                "s1": reset_pose["s1"],
                "s2": reset_pose["s2"],
                "s3": reset_pose["s3"],
                "s4": reset_pose["s4"],
                "s5": reset_pose["s5"],
                "s6": reset_pose["s6"],
            }
            return jsonify({"success": True, "positions": initial_positions, "source": "database"})
        else:
            # Fallback to SERVO_CONFIG defaults
            initial_positions = {
                "s1": SERVO_CONFIG["b"]["start_angle"],
                "s2": SERVO_CONFIG["s"]["start_angle"],
                "s3": SERVO_CONFIG["e"]["start_angle"],
                "s4": SERVO_CONFIG["f"]["start_angle"],
                "s5": SERVO_CONFIG["w"]["start_angle"],
                "s6": SERVO_CONFIG["g"]["start_angle"],
            }
            return jsonify({"success": True, "positions": initial_positions, "source": "config"})

    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


# --- CLEANUP ---
def cleanup():
    """Cleanup GPIO and servos on shutdown"""
    print("\n[SHUTDOWN] Cleaning up...")
    try:
        # Save current position before shutting down
        save_current_position()

        if pca is not None:
            for i in range(16):
                pca.channels[i].duty_cycle = 0
        if oe_pin:
            oe_pin.on()  # Disable motors
            oe_pin.close()
        print("[CLEANUP] Done.")
    except Exception as e:
        print(f"[CLEANUP ERROR] {e}")


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n[SIGNAL] Interrupt received, saving state...")
    save_current_position()
    cleanup()
    sys.exit(0)


# --- MAIN ---
if __name__ == "__main__":
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    atexit.register(save_current_position)  # Save on normal exit too

    try:
        init_database()
        init_admin()  # Create admin account
        init_hardware()

        # Load last known position
        last_pos = load_last_position()
        if last_pos:
            print("[STATE] Last known positions will be used as defaults")
            # Update SERVO_CONFIG with last positions
            SERVO_CONFIG["s"]["start_angle"] = last_pos.get("s2", 94)
            SERVO_CONFIG["e"]["start_angle"] = last_pos.get("s3", 94)
            SERVO_CONFIG["f"]["start_angle"] = last_pos.get("s4", 90)
            SERVO_CONFIG["w"]["start_angle"] = last_pos.get("s5", 60)
            SERVO_CONFIG["g"]["start_angle"] = last_pos.get("s6", 45)

        print("\n" + "=" * 50)
        print("  🌐 Web UI available at http://localhost:5000")
        print("=" * 50 + "\n")
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        cleanup()
    except Exception as e:
        print(f"[ERROR] {e}")
        cleanup()
