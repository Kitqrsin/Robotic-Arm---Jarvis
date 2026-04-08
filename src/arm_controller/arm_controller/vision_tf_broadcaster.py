#!/usr/bin/env python3
"""
Vision TF Broadcaster Node - Detects bartender objects and publishes TF2 frames.

Subscribes to /camera/image_raw, runs YOLOv8 detection for cups, bottles, and 
wine glasses, estimates their 3D position using monocular depth estimation 
(known object sizes), and broadcasts TF2 frames relative to camera_link.

The TF tree will be: base_link -> camera_link -> detected_object_N

Requirements:
    - ultralytics (YOLOv8)
    - opencv-python
    - tf2_ros
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
import json
import math
import os

try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class VisionTFBroadcaster(Node):
    """
    Detects bartender objects and broadcasts their positions as TF2 frames.
    
    Uses monocular depth estimation based on known object sizes for Pi Camera V2.
    """

    # COCO class IDs for bartender-relevant objects
    BARTENDER_CLASSES = {
        39: 'bottle',
        40: 'wine_glass', 
        41: 'cup',
        43: 'fork',
        44: 'knife',
        45: 'spoon',
        46: 'bowl',
    }

    # Known real-world heights (metres) for depth estimation
    # These are approximate averages - adjust based on your actual items
    KNOWN_HEIGHTS = {
        'bottle': 0.28,        # ~28cm tall (standard wine/liquor bottle)
        'wine_glass': 0.20,    # ~20cm tall
        'cup': 0.10,           # ~10cm tall (standard cup/mug)
        'fork': 0.19,
        'knife': 0.22,
        'spoon': 0.17,
        'bowl': 0.08,          # height when viewed from side
    }
    
    # Known widths (metres) as backup for depth estimation
    KNOWN_WIDTHS = {
        'bottle': 0.08,        # ~8cm diameter
        'wine_glass': 0.08,
        'cup': 0.08,
        'fork': 0.025,
        'knife': 0.025,
        'spoon': 0.04,
        'bowl': 0.15,
    }

    def __init__(self):
        super().__init__('vision_tf_broadcaster')

        # --- Parameters ---
        self.declare_parameter('model_path', '/workspace/yolov8n.pt')
        self.declare_parameter('confidence', 0.50)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('detect_interval', 3)  # Process every Nth frame
        self.declare_parameter('max_detections', 5)
        
        # Camera intrinsics for Pi Camera V2 (default values, can be overridden)
        # Pi Camera V2: 3280x2464 sensor, but usually used at 640x480
        self.declare_parameter('focal_length_px', 500.0)  # Approximate for 640x480
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # Camera mounting position relative to base_link (metres)
        self.declare_parameter('camera_x', 0.0)    # Forward from base
        self.declare_parameter('camera_y', 0.0)    # Left from base
        self.declare_parameter('camera_z', 0.30)   # Height above base
        self.declare_parameter('camera_pitch', -0.3)  # Radians, tilted down
        
        # TF frame names
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')

        # Load parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value
        self.device = self.get_parameter('device').value
        self.detect_interval = self.get_parameter('detect_interval').value
        self.max_det = self.get_parameter('max_detections').value
        
        self.focal_length = self.get_parameter('focal_length_px').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        
        self.camera_x = self.get_parameter('camera_x').value
        self.camera_y = self.get_parameter('camera_y').value
        self.camera_z = self.get_parameter('camera_z').value
        self.camera_pitch = self.get_parameter('camera_pitch').value
        
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # Compute principal point (image center)
        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0

        # --- Load YOLO model ---
        self.model = None
        if YOLO_AVAILABLE:
            # Try multiple paths for the model
            model_paths = [
                self.model_path,
                '/workspace/yolov8n.pt',
                os.path.expanduser('~/arm_project/Robotic-Arm---Jarvis/yolov8n.pt'),
                'yolov8n.pt',  # Will download if not found
            ]
            for path in model_paths:
                try:
                    if os.path.exists(path) or path == 'yolov8n.pt':
                        self.model = YOLO(path)
                        self.get_logger().info(f'Loaded YOLO model: {path}')
                        break
                except Exception as e:
                    self.get_logger().debug(f'Failed to load {path}: {e}')
            
            if self.model is None:
                self.get_logger().error('Could not load any YOLO model!')
        else:
            self.get_logger().error('ultralytics not installed! pip install ultralytics')

        # --- TF Broadcasters ---
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish static transform: base_link -> camera_link
        self._publish_camera_static_tf()

        # --- Publishers ---
        self.detections_pub = self.create_publisher(
            String, '/vision/detected_objects', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/vision/object_markers', 10
        )
        self.annotated_pub = self.create_publisher(
            Image, '/vision/image_annotated', 10
        )

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )

        # State
        self.frame_count = 0
        self.detected_objects = {}  # name -> {x, y, z, confidence, last_seen}
        self.object_counters = {}   # class_name -> count for unique naming
        
        self.get_logger().info(
            f'Vision TF Broadcaster started\n'
            f'  Model: {self.model_path}\n'
            f'  Confidence: {self.confidence}\n'
            f'  Camera frame: {self.camera_frame}\n'
            f'  Focal length: {self.focal_length}px'
        )

    def _publish_camera_static_tf(self):
        """Publish static transform from base_link to camera_link."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.camera_frame
        
        t.transform.translation.x = self.camera_x
        t.transform.translation.y = self.camera_y
        t.transform.translation.z = self.camera_z
        
        # Camera is pitched down - convert to quaternion
        # Rotation around Y axis (pitch)
        cy = math.cos(self.camera_pitch / 2)
        sy = math.sin(self.camera_pitch / 2)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = sy
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = cy
        
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f'Published static TF: {self.base_frame} -> {self.camera_frame}'
        )

    def camera_info_callback(self, msg: CameraInfo):
        """Update camera intrinsics from CameraInfo if available."""
        if msg.k[0] > 0:  # fx
            self.focal_length = msg.k[0]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.image_width = msg.width
            self.image_height = msg.height
            self.get_logger().info(
                f'Updated camera intrinsics: fx={self.focal_length:.1f}, '
                f'cx={self.cx:.1f}, cy={self.cy:.1f}',
                throttle_duration_sec=30.0
            )

    def image_callback(self, msg: Image):
        """Process incoming camera frame."""
        self.frame_count += 1
        
        # Only process every Nth frame
        if self.frame_count % self.detect_interval != 0:
            return
        
        if self.model is None:
            return
        
        # Convert ROS Image to numpy
        frame = self._ros_image_to_numpy(msg)
        if frame is None:
            return
        
        # Update image dimensions
        self.image_height, self.image_width = frame.shape[:2]
        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0
        
        # Run YOLO inference
        try:
            results = self.model.predict(
                frame,
                conf=self.confidence,
                device=self.device,
                max_det=self.max_det,
                verbose=False,
                classes=list(self.BARTENDER_CLASSES.keys())  # Filter to bartender objects
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference error: {e}')
            return
        
        # Process detections
        detections = self._process_detections(results, frame.shape, msg.header.stamp)
        
        # Broadcast TF for each detection
        self._broadcast_object_tfs(detections, msg.header.stamp)
        
        # Publish markers for RViz visualization
        self._publish_markers(detections, msg.header.stamp)
        
        # Publish detection info as JSON
        self._publish_detections_json(detections, msg.header)
        
        # Publish annotated image
        if OPENCV_AVAILABLE and detections:
            annotated = self._draw_detections(frame.copy(), detections)
            self._publish_annotated_image(annotated, msg.header)

    def _process_detections(self, results, image_shape, stamp):
        """Parse YOLO results and estimate 3D positions."""
        detections = []
        
        if not results or len(results) == 0:
            return detections
        
        result = results[0]
        if result.boxes is None:
            return detections
        
        h, w = image_shape[:2]
        
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            
            # Get class name
            if cls_id in self.BARTENDER_CLASSES:
                cls_name = self.BARTENDER_CLASSES[cls_id]
            else:
                cls_name = result.names.get(cls_id, f'object_{cls_id}')
            
            # Calculate bounding box center and size
            bbox_cx = (x1 + x2) / 2.0
            bbox_cy = (y1 + y2) / 2.0
            bbox_w = x2 - x1
            bbox_h = y2 - y1
            
            # Estimate depth using known object size
            depth = self._estimate_depth(cls_name, bbox_w, bbox_h)
            
            # Convert pixel coordinates to 3D camera frame coordinates
            # Using pinhole camera model: X = (u - cx) * Z / fx
            x_cam = (bbox_cx - self.cx) * depth / self.focal_length
            y_cam = (bbox_cy - self.cy) * depth / self.focal_length
            z_cam = depth
            
            # Generate unique object ID
            if cls_name not in self.object_counters:
                self.object_counters[cls_name] = 0
            self.object_counters[cls_name] += 1
            obj_id = f"{cls_name}_{self.object_counters[cls_name] % 10}"
            
            detections.append({
                'id': obj_id,
                'class_name': cls_name,
                'class_id': cls_id,
                'confidence': conf,
                'bbox_px': {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2},
                'center_px': {'x': bbox_cx, 'y': bbox_cy},
                'size_px': {'w': bbox_w, 'h': bbox_h},
                'position_camera': {'x': x_cam, 'y': y_cam, 'z': z_cam},
                'depth_m': depth,
            })
        
        # Sort by confidence
        detections.sort(key=lambda d: d['confidence'], reverse=True)
        
        # Keep only top N
        return detections[:self.max_det]

    def _estimate_depth(self, class_name: str, bbox_width_px: float, bbox_height_px: float) -> float:
        """
        Estimate object depth using known real-world sizes.
        
        Uses the formula: depth = (real_size * focal_length) / pixel_size
        
        For Pi Camera V2, this gives reasonable estimates for objects 0.3-2m away.
        """
        # Use height as primary estimator (usually more reliable)
        if class_name in self.KNOWN_HEIGHTS and bbox_height_px > 10:
            real_height = self.KNOWN_HEIGHTS[class_name]
            depth_from_height = (real_height * self.focal_length) / bbox_height_px
        else:
            depth_from_height = 1.0  # Default 1 meter
        
        # Use width as secondary estimator
        if class_name in self.KNOWN_WIDTHS and bbox_width_px > 10:
            real_width = self.KNOWN_WIDTHS[class_name]
            depth_from_width = (real_width * self.focal_length) / bbox_width_px
        else:
            depth_from_width = depth_from_height
        
        # Average the two estimates (weighted towards height)
        depth = 0.7 * depth_from_height + 0.3 * depth_from_width
        
        # Clamp to reasonable range (0.1m to 3m)
        depth = max(0.10, min(3.0, depth))
        
        return depth

    def _broadcast_object_tfs(self, detections, stamp):
        """Broadcast TF2 transforms for each detected object."""
        transforms = []
        
        for det in detections:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.camera_frame
            t.child_frame_id = f"detected_{det['id']}"
            
            # Position in camera frame
            # Note: Camera coordinate system is typically:
            #   X = right, Y = down, Z = forward (into the scene)
            # We convert to robot-style coordinates:
            #   X = forward, Y = left, Z = up
            pos = det['position_camera']
            t.transform.translation.x = pos['z']   # Camera Z -> Robot X (forward)
            t.transform.translation.y = -pos['x']  # Camera X -> Robot -Y (right to left)
            t.transform.translation.z = -pos['y']  # Camera Y -> Robot -Z (down to up)
            
            # No rotation (object frame aligned with camera frame)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            transforms.append(t)
        
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

    def _publish_markers(self, detections, stamp):
        """Publish RViz markers for visualization."""
        marker_array = MarkerArray()
        
        for i, det in enumerate(detections):
            # Sphere marker for object position
            marker = Marker()
            marker.header.frame_id = self.camera_frame
            marker.header.stamp = stamp
            marker.ns = 'detected_objects'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            pos = det['position_camera']
            marker.pose.position.x = pos['z']
            marker.pose.position.y = -pos['x']
            marker.pose.position.z = -pos['y']
            marker.pose.orientation.w = 1.0
            
            # Size based on object class
            size = self.KNOWN_WIDTHS.get(det['class_name'], 0.08)
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = self.KNOWN_HEIGHTS.get(det['class_name'], 0.15)
            
            # Color by class
            if det['class_name'] == 'bottle':
                marker.color.r, marker.color.g, marker.color.b = 0.2, 0.8, 0.2
            elif det['class_name'] == 'cup':
                marker.color.r, marker.color.g, marker.color.b = 0.8, 0.4, 0.1
            elif det['class_name'] == 'wine_glass':
                marker.color.r, marker.color.g, marker.color.b = 0.6, 0.2, 0.8
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.5, 0.5, 0.5
            marker.color.a = 0.7
            
            marker.lifetime.sec = 1  # Auto-delete after 1 second
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'object_labels'
            text_marker.id = i + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pos['z']
            text_marker.pose.position.y = -pos['x']
            text_marker.pose.position.z = -pos['y'] + 0.15  # Above the object
            text_marker.scale.z = 0.05
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{det['class_name']}\n{det['depth_m']:.2f}m"
            text_marker.lifetime.sec = 1
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)

    def _publish_detections_json(self, detections, header):
        """Publish detections as JSON string."""
        msg = String()
        msg.data = json.dumps({
            'timestamp': header.stamp.sec + header.stamp.nanosec * 1e-9,
            'frame_id': header.frame_id,
            'count': len(detections),
            'detections': [
                {
                    'id': d['id'],
                    'class': d['class_name'],
                    'confidence': round(d['confidence'], 3),
                    'tf_frame': f"detected_{d['id']}",
                    'position': {
                        'x': round(d['position_camera']['z'], 3),
                        'y': round(-d['position_camera']['x'], 3),
                        'z': round(-d['position_camera']['y'], 3),
                    },
                    'depth_m': round(d['depth_m'], 3),
                }
                for d in detections
            ]
        })
        self.detections_pub.publish(msg)

    def _draw_detections(self, frame, detections):
        """Draw bounding boxes and depth info on frame."""
        for det in detections:
            bb = det['bbox_px']
            x1, y1, x2, y2 = int(bb['x1']), int(bb['y1']), int(bb['x2']), int(bb['y2'])
            
            # Color by class
            if det['class_name'] == 'bottle':
                color = (50, 200, 50)
            elif det['class_name'] == 'cup':
                color = (200, 100, 25)
            elif det['class_name'] == 'wine_glass':
                color = (150, 50, 200)
            else:
                color = (128, 128, 128)
            
            # Draw box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # Label with depth
            label = f"{det['class_name']} {det['confidence']:.0%} @ {det['depth_m']:.2f}m"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 4, y1), color, -1)
            cv2.putText(frame, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Draw TF frame name
            tf_label = f"TF: detected_{det['id']}"
            cv2.putText(frame, tf_label, (x1, y2 + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        return frame

    def _publish_annotated_image(self, frame_rgb, header):
        """Publish annotated frame as ROS Image."""
        msg = Image()
        msg.header = header
        msg.height = frame_rgb.shape[0]
        msg.width = frame_rgb.shape[1]
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = frame_rgb.shape[1] * 3
        msg.data = frame_rgb.tobytes()
        self.annotated_pub.publish(msg)

    def _ros_image_to_numpy(self, msg: Image):
        """Convert ROS Image to numpy array (RGB)."""
        try:
            if msg.encoding == 'rgb8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
            elif msg.encoding == 'bgr8':
                bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
                frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            else:
                self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
                return None
            return frame
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = VisionTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down vision TF broadcaster...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
