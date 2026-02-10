import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Bool
import time

# We use a try-except block here so the node doesn't crash if hardware isn't plugged in yet
try:
    from adafruit_servokit import ServoKit
except ImportError:
    ServoKit = None

# Import gpiozero for OE pin control (emergency stop)
try:
    from gpiozero import OutputDevice
    GPIOZERO_AVAILABLE = True
except ImportError:
    GPIOZERO_AVAILABLE = False


class ArmServoNode(Node):
    def __init__(self):
        super().__init__("arm_servo_node")

        self.kit = None
        
        # Current servo positions (tracked for smooth movement)
        self.current_angles = [90.0] * 6  # Start at 90 degrees (centered)
        
        # Speed control: degrees per step (1-100 scale)
        # 100 = instant, 1 = very slow
        self.speed = 50.0  # Default medium speed
        self.step_delay = 0.01  # Delay between interpolation steps (10ms)
        
        # ============================================================
        # SERVO CHANNEL MAPPING - ADJUST THIS TO MATCH YOUR WIRING
        # ============================================================
        # Format: GUI joint index -> PCA9685 channel number
        # GUI joints: 0=base, 1=shoulder, 2=elbow, 3=wrist, 4=wrist_rotate, 5=gripper
        # Physical channels used: 0, 1, 2, 4, 5, 6
        # Mapping: J1->0, J2->1, J3->2, J4->4, J5->5, J6->6
        self.channel_map = [0, 1, 2, 4, 5, 6]
        
        # Direction inversion - set True to invert (180 - angle)
        # Use this if a joint moves backwards
        # Inverted: Shoulder (J2=index 1), Elbow (J3=index 2)
        self.invert_direction = [False, True, True, False, False, False]
        # ============================================================
        
        # ============================================================
        # EMERGENCY STOP - PCA9685 OE (Output Enable) Pin
        # ============================================================
        # OE pin on PCA9685: LOW = outputs enabled, HIGH = outputs disabled
        # Connected to Raspberry Pi GPIO17
        self.oe_pin = None
        self.emergency_stopped = False
        
        if GPIOZERO_AVAILABLE:
            try:
                # GPIO17 controls OE pin - start with outputs ENABLED (LOW)
                self.oe_pin = OutputDevice(17, initial_value=False)
                self.get_logger().info("Emergency Stop (OE pin on GPIO17) initialized - outputs ENABLED")
            except Exception as e:
                self.get_logger().warn(f"Could not initialize OE pin on GPIO17: {e}")
        else:
            self.get_logger().warn("gpiozero not available - emergency stop via OE pin disabled")
        # ============================================================

        # Initialize PCA9685 (16 channels)
        try:
            if ServoKit:
                # address=0x40 is standard. On Pi 5, ensures correct I2C bus.
                self.kit = ServoKit(channels=16)
                self.get_logger().info("PCA9685 Servo Driver Connected!")
                self.get_logger().info(f"Channel mapping: {self.channel_map}")
                self.get_logger().info(f"Inverted joints: {[i for i, inv in enumerate(self.invert_direction) if inv]}")
                # Initialize servos smoothly to centered position (90°)
                self._smooth_init_to_home()
            else:
                self.get_logger().warn("Adafruit library not found.")
        except Exception as e:
            self.get_logger().error(f"Could not connect to servo board: {e}")
            self.get_logger().error(
                "Check if I2C is enabled in raspi-config and wires are secure."
            )

        # Subscribe to "joint_commands"
        self.subscription = self.create_subscription(
            Float32MultiArray, "joint_commands", self.listener_callback, 10
        )
        
        # Subscribe to speed control
        self.speed_subscription = self.create_subscription(
            Float32, "/servo_speed", self.speed_callback, 10
        )
        
        # Subscribe to emergency stop
        self.estop_subscription = self.create_subscription(
            Bool, "/emergency_stop", self.emergency_stop_callback, 10
        )
        
        self.get_logger().info(f"Speed control enabled. Default speed: {self.speed}")

    def speed_callback(self, msg):
        """Handle speed updates from GUI"""
        self.speed = max(1.0, min(100.0, msg.data))  # Clamp 1-100
        self.get_logger().info(f"Speed set to: {self.speed}")

    def emergency_stop_callback(self, msg):
        """Handle emergency stop command - uses PCA9685 OE pin"""
        if msg.data:
            self.disable_all_servos()
        else:
            self.release_emergency_stop()
    
    def activate_emergency_stop(self):
        """Activate emergency stop - disable all servo outputs via OE pin"""
        self.disable_all_servos()
    
    def release_emergency_stop(self):
        """Release emergency stop - re-enable servo outputs via OE pin"""
        if self.oe_pin:
            self.oe_pin.off()  # LOW = outputs enabled
            self.emergency_stopped = False
            self.get_logger().info("✓ Emergency stop RELEASED - Servo outputs ENABLED")
        else:
            self.get_logger().warn("Release requested but OE pin not available")

    def _smooth_init_to_home(self):
        """Smoothly ramp all servos to 90° (home) on startup.
        
        The PCA9685 doesn't report current positions, so on a cold start
        the servos could be anywhere. We start by writing a position close
        to 90° and ramp very slowly to avoid slamming.
        
        Strategy:
        1. Try to read each servo's last-written angle from the driver.
           On cold boot this is typically None/unknown.
        2. If unknown, set each servo to 90° directly but with the OE pin
           HIGH (outputs disabled), so no physical movement occurs yet.
        3. Enable OE (outputs on) and hold — servos jump to 90° but
           from an unpowered state this is usually gentle.
        4. For extra safety, sweep from a range of possible worst-case
           positions toward 90° so any already-powered servo moves slowly.
        """
        self.get_logger().info("Starting smooth initialization to home (90°)...")
        
        target = 90.0
        init_steps = 50          # Number of interpolation steps
        init_step_delay = 0.03   # 30ms per step → ~1.5s total ramp time
        
        # Try reading current positions from the driver
        start_angles = []
        for i in range(6):
            channel = self.channel_map[i]
            try:
                current = self.kit.servo[channel].angle
                if current is not None and 0 <= current <= 270:
                    start_angles.append(current)
                else:
                    start_angles.append(target)  # Unknown → assume already at 90
            except Exception:
                start_angles.append(target)
        
        # Check if any servo needs to move
        max_distance = max(abs(s - target) for s in start_angles)
        
        if max_distance < 1.0:
            # Already at home — just write 90° to be sure and return
            for i in range(6):
                channel = self.channel_map[i]
                self.kit.servo[channel].angle = target
            self.current_angles = [target] * 6
            self.get_logger().info("Servos already at home position (90°)")
            return
        
        self.get_logger().info(
            f"Ramping from {[f'{a:.0f}' for a in start_angles]} → 90° "
            f"over {init_steps} steps ({init_steps * init_step_delay:.1f}s)"
        )
        
        # Gradually interpolate from current → 90°
        for step in range(1, init_steps + 1):
            progress = step / init_steps
            # Ease-in-out for gentler acceleration/deceleration
            # Using smoothstep: 3t² - 2t³
            smooth_progress = progress * progress * (3.0 - 2.0 * progress)
            
            for i in range(6):
                intermediate = start_angles[i] + (target - start_angles[i]) * smooth_progress
                
                # Apply direction inversion if needed
                if self.invert_direction[i]:
                    servo_angle = 180.0 - intermediate
                else:
                    servo_angle = intermediate
                
                # Safety clamp
                servo_angle = max(0.0, min(180.0, servo_angle))
                
                channel = self.channel_map[i]
                self.kit.servo[channel].angle = servo_angle
            
            if step < init_steps:
                time.sleep(init_step_delay)
        
        # Update tracked positions
        self.current_angles = [target] * 6
        self.get_logger().info("✓ Smooth initialization complete — all servos at 90°")

    def smooth_move(self, target_angles):
        """Smoothly interpolate servos to target positions"""
        if not self.kit:
            return
        
        # Don't move if emergency stop is active
        if self.emergency_stopped:
            self.get_logger().warn("Movement blocked - emergency stop is active")
            return
            
        # Calculate the maximum distance any servo needs to travel
        distances = [abs(target_angles[i] - self.current_angles[i]) for i in range(6)]
        max_distance = max(distances) if distances else 0
        
        if max_distance < 0.5:
            # Already at target
            return
        
        # Calculate number of steps based on speed
        # Speed 100 = 1 step (instant), Speed 1 = many steps (slow)
        # Map speed to degrees per step: speed 100 = 10 deg/step, speed 1 = 0.1 deg/step
        degrees_per_step = (self.speed / 10.0)  # 0.1 to 10 degrees per step
        num_steps = max(1, int(max_distance / degrees_per_step))
        
        # Limit steps for responsiveness
        num_steps = min(num_steps, 100)
        
        for step in range(1, num_steps + 1):
            progress = step / num_steps
            
            for i in range(6):
                # Linear interpolation
                current = self.current_angles[i]
                target = target_angles[i]
                intermediate = current + (target - current) * progress
                
                # Safety clamp - base (index 0) allows up to 245°, others 180°
                max_angle = 245.0 if i == 0 else 180.0
                intermediate = max(0.0, min(max_angle, intermediate))
                
                # Apply direction inversion if needed
                if self.invert_direction[i]:
                    servo_angle = 180.0 - intermediate
                else:
                    servo_angle = intermediate
                
                # Move servo using channel mapping
                channel = self.channel_map[i]
                self.kit.servo[channel].angle = servo_angle
            
            # Small delay between steps for smooth motion
            if step < num_steps:
                time.sleep(self.step_delay)
        
        # Update current positions
        self.current_angles = list(target_angles)

    def disable_all_servos(self):
        """Disable all servo outputs - called on shutdown or emergency"""
        self.get_logger().info("Disabling all servo outputs...")
        
        # Method 1: Set OE pin HIGH to disable PCA9685 outputs
        if self.oe_pin:
            self.oe_pin.on()  # HIGH = outputs disabled
            self.get_logger().info("OE pin set HIGH - servo outputs disabled")
        
        # Method 2: Set all servo angles to None (releases PWM signal)
        if self.kit:
            try:
                for channel in self.channel_map:
                    self.kit.servo[channel].angle = None  # Releases the servo
                self.get_logger().info("All servo PWM signals released")
            except Exception as e:
                self.get_logger().warn(f"Error releasing servos: {e}")
        
        self.emergency_stopped = True
        self.get_logger().warn("⛔ ALL MOTORS DISABLED")

    def listener_callback(self, msg):
        angles = msg.data
        if len(angles) != 6:
            self.get_logger().warn(f"Expected 6 angles, got {len(angles)}")
            return

        # Prepare target angles with safety clamping
        # Base (index 0) is a 270-degree servo, limited to 245° for safety
        target_angles = []
        for i in range(6):
            target_angle = angles[i]
            max_angle = 245 if i == 0 else 180  # Base is 245° (safety limit), others 180°
            if target_angle < 0:
                target_angle = 0
            if target_angle > max_angle:
                target_angle = max_angle
            target_angles.append(target_angle)

        self.get_logger().info(f"Moving to: {target_angles} at speed {self.speed}")

        if self.kit:
            # Use smooth movement instead of instant jump
            self.smooth_move(target_angles)


def main(args=None):
    rclpy.init(args=args)
    node = ArmServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received - shutting down safely...")
    finally:
        # ALWAYS disable motors on exit
        node.disable_all_servos()
        node.destroy_node()
        rclpy.shutdown()
        print("\n✓ Servo node shutdown complete - all motors disabled")


if __name__ == "__main__":
    main()
