# 🤖 Jarvis Arm - 5-Axis Robot Arm Controller

A web-based control system for a 5-axis robot arm powered by Raspberry Pi, featuring real-time servo control, position saving, pose chaining, and secure authentication.

## 📋 Features

### Core Functionality
- **Real-time Servo Control**: Individual control of 5 servos (S2-S6) with live position feedback
- **Per-Servo Speed Control**: Adjustable movement speed (10-500ms delay) for each servo independently
- **Motor Enable/Disable**: Hardware-level motor control via GPIO output enable (OE) pin
- **Emergency Stop**: Always-accessible stop button for immediate movement interruption
- **Position Persistence**: Automatic saving of last position on shutdown (Ctrl+C)

### Advanced Features
- **Custom Pose Management**: Save and recall named positions with SQLite database storage
- **Pose Chaining System**: Execute up to 5 poses sequentially with 500ms inter-pose delay
- **Per-Motor Movement Locking**: Individual servo locks prevent conflicts during gradual movements
- **Duplicate Pose Support**: Chain system allows the same pose multiple times in sequence

### Security
- **Session-Based Authentication**: Secure login system with bcrypt password hashing
- **Admin-Only Registration**: User creation restricted to admin accounts
- **Protected API Routes**: All hardware control endpoints require authentication
- **Default Admin Account**: Automatic creation of admin user on first startup

### User Interface
- **Modern Web Interface**: Responsive design with Tailwind CSS
- **Real-time Status Indicators**: Live connection and motor status display
- **Interactive Sliders**: Onchange-triggered position control for precise movements
- **Visual Feedback**: Color-coded status, movement locks, and chain queue display

## 🔧 Hardware Requirements

- **Microcontroller**: Raspberry Pi (any model with GPIO and I2C)
- **Servo Driver**: PCA9685 16-Channel PWM/Servo Driver (I2C address: 0x40)
- **Servos**: 5 standard servos connected to channels 0-4
- **GPIO Control**: GPIO 17 for output enable (OE) pin control
- **Power Supply**: Adequate power for 5 servos (recommend 5-6V, 5A+)

### Wiring Configuration
- **I2C**: SDA/SCL connected to Raspberry Pi I2C pins
- **OE Pin**: PCA9685 OE pin connected to GPIO 17
- **Servos**: 
  - S2 → Channel 1
  - S3 → Channel 2
  - S4 → Channel 3
  - S5 → Channel 4
  - S6 → Channel 5

## 📦 Software Dependencies

### Python Packages
```bash
Flask==3.x
Flask-CORS
gpiozero==2.0.1
adafruit-circuitpython-pca9685
werkzeug
sqlite3 (standard library)
```

### System Requirements
- Python 3.11+
- I2C enabled on Raspberry Pi
- SQLite3

## 🚀 Installation

### 1. Clone Repository
```bash
git clone <repository-url>
cd robot_arm_ws
```

### 2. Install Dependencies
```bash
pip install Flask Flask-CORS gpiozero adafruit-circuitpython-pca9685
```

### 3. Enable I2C on Raspberry Pi
```bash
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
sudo reboot
```

### 4. Verify Hardware Connection
```bash
i2cdetect -y 1
# Should show device at address 0x40
```

## 🎮 Usage

### Starting the Server
```bash
cd src/arm_controller/arm_controller
python3 app.py
```

The server will:
- Initialize SQLite database (robot_poses.db)
- Create admin account (username: `username`, password: `user_password`)
- Initialize PCA9685 servo driver
- Start Flask server on `http://0.0.0.0:5000`

### First Time Setup
1. Navigate to `http://<raspberry-pi-ip>:5000`
2. Log in with admin credentials (username / user_password)
3. Enable motors using the "ENABLE MOTORS" button
4. Control servos using position sliders
5. Create additional users via admin panel (accessible to admin only)

### Saving Custom Poses
1. Move servos to desired position
2. Click "Save Current Position"
3. Enter a name for the pose
4. Pose is saved to database

### Using Pose Chaining
1. Click saved poses to add them to chain queue (max 5)
2. Poses can be added multiple times (duplicates allowed)
3. Click "Execute Chain" to run sequence
4. Use "Stop Chain" for emergency stop during execution

### Graceful Shutdown
Press `Ctrl+C` to trigger cleanup:
- Current position saved to `last_position.json`
- All servo channels set to 0 duty cycle
- Motors disabled via OE pin

## 📁 Project Structure

```
robot_arm_ws/
├── src/
│   ├── requirements.txt
│   └── arm_controller/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       └── arm_controller/
│           ├── __init__.py
│           ├── app.py                 # Flask server & hardware control
│           ├── servo_node.py          # (Legacy ROS2 node)
│           ├── stop_servos.py         # Emergency stop utility
│           ├── test.py                # Hardware test script
│           ├── robot_poses.db         # SQLite database (auto-created)
│           ├── last_position.json     # Position persistence (auto-created)
│           ├── templates/
│           │   ├── index.html         # Main control interface
│           │   ├── login.html         # Authentication page
│           │   └── register.html      # Admin user creation page
│           └── static/
│               └── js/
│                   └── app.js         # Frontend control logic
```

## 🔐 Security Features

### Authentication System
- **Password Hashing**: Werkzeug bcrypt implementation (SHA-256)
- **Session Management**: Flask session with secure secret key
- **Admin Privileges**: Role-based access control with `is_admin` flag
- **Protected Routes**: All `/api/*` endpoints require `@login_required` decorator

### Default Credentials
- **Username**: username
- **Password**: user_password
- ⚠️ **IMPORTANT**: Change default password after first login in production!

### User Creation
- Only admin users can create new accounts via `/register` route
- New users created without admin privileges (`is_admin = 0`)
- Username uniqueness enforced at database level

## 🗄️ Database Schema

### Users Table
```sql
CREATE TABLE users (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    username TEXT UNIQUE NOT NULL,
    password_hash TEXT NOT NULL,
    is_admin BOOLEAN DEFAULT 0,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### Poses Table
```sql
CREATE TABLE poses (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    s2 INTEGER NOT NULL,
    s3 INTEGER NOT NULL,
    s4 INTEGER NOT NULL,
    s5 INTEGER NOT NULL,
    s6 INTEGER NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

## ⚙️ Configuration

### Servo Angle Limits
Edit in `app.py`:
```python
SERVO_MIN = 0    # Minimum angle (degrees)
SERVO_MAX = 180  # Maximum angle (degrees)
```

### Home Position
Default angles (0° position):
```python
HOME_POSITIONS = {
    's2': 94,
    's3': 94,
    's4': 72,
    's5': 60,
    's6': 45
}
```

### Movement Speed
- **Range**: 10ms - 500ms per degree
- **Default**: 100ms per degree per servo
- **Adjustable**: Individual speed sliders in web interface

### PWM Frequency
```python
pca.frequency = 50  # 50Hz for standard servos
```

## 🌐 API Endpoints

All endpoints require authentication via session cookie.

### Hardware Control
- `POST /api/servo` - Set single servo position
- `POST /api/servos/batch` - Set multiple servos simultaneously
- `POST /api/emergency-stop` - Emergency stop all servos
- `POST /api/oe/toggle` - Enable/disable motors via OE pin
- `GET /api/status` - Get current servo positions and motor status

### Pose Management
- `GET /api/poses` - Get all saved poses
- `POST /api/poses/save` - Save current position as named pose
- `DELETE /api/poses/<id>` - Delete saved pose
- `GET /api/last-position` - Get last saved position from file

### Authentication
- `GET /login` - Login page
- `POST /login` - Login handler
- `GET /register` - User creation page (admin only)
- `POST /register` - User creation handler (admin only)
- `GET /logout` - Logout and clear session

## 🐛 Troubleshooting

### I2C Device Not Found
```bash
sudo i2cdetect -y 1
# If empty, check wiring and enable I2C in raspi-config
```

### Permission Denied on GPIO
```bash
sudo usermod -a -G gpio $USER
# Logout and login again
```

### Database Migration Error
If you see "table users has no column named is_admin":
- The auto-migration will run on next startup
- Manual fix: `ALTER TABLE users ADD COLUMN is_admin BOOLEAN DEFAULT 0;`

### Servo Not Moving
1. Check "ENABLE MOTORS" is active (yellow button)
2. Verify OE pin connection to GPIO 17
3. Confirm servo power supply is adequate
4. Check PCA9685 channel connections

### Authentication Loop
- Clear browser cookies
- Check Flask session secret key is set
- Restart server to regenerate session

## 📝 Development Notes

### Movement System
- Gradual movement: 1° steps with per-servo delay
- Per-servo locking prevents simultaneous movements on same motor
- `onchange` event trigger (movement on slider release, not during drag)

### Frontend State Management
```javascript
isMoving = {s2: false, s3: false, s4: false, s5: false, s6: false}
servoSpeeds = {s2: 100, s3: 100, s4: 100, s5: 100, s6: 100}
chainQueue = []  // Max 5 poses
```

### Signal Handlers
- `SIGINT (Ctrl+C)`: Save position and cleanup
- `SIGTERM`: Graceful shutdown
- `atexit`: Cleanup hook for normal exit

## 🔄 Version History

### v1.0.0 (Current)
- ✅ Per-motor movement locking
- ✅ Individual servo speed controls
- ✅ Position persistence with signal handlers
- ✅ Pose chaining system (max 5, duplicates allowed)
- ✅ Session-based authentication
- ✅ Admin-only user registration
- ✅ Motor state-dependent UI locking

## 📄 License

TODO: Add license information

## 👤 Author

**Kristiyan**

## 🤝 Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open Pull Request

## ⚠️ Safety Warnings

- Always test in a safe environment before full operation
- Emergency stop button should always be accessible
- Monitor servo temperatures during extended use
- Ensure adequate power supply to prevent brownouts
- Keep workspace clear of obstructions
- Never disable motor safety interlocks in production

## 📞 Support

For issues and questions, please open an issue on GitHub.

---

**Made with ❤️ for sigma grindset bob the builder aurafarmers**
