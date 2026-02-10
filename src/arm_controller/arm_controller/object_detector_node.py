#!/usr/bin/env python3
"""
Object Detector Node - Runs YOLOv8 on camera frames
Subscribes to /camera/image_raw, publishes detections to /detected_objects.

Uses ultralytics YOLOv8n (nano) for fast inference on Raspberry Pi.
On first run it auto-downloads the model weights (~6 MB).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import numpy as np
import json
import time

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


class ObjectDetectorNode(Node):
    """Detects objects in camera frames using YOLOv8."""

    def __init__(self):
        super().__init__('object_detector_node')

        # --- Parameters ---
        self.declare_parameter('model', 'yolov8n.pt')         # YOLO model file
        self.declare_parameter('confidence', 0.45)             # Min detection confidence
        self.declare_parameter('device', 'cpu')                # 'cpu' or '0' for GPU
        self.declare_parameter('max_detections', 10)           # Max objects per frame
        self.declare_parameter('detect_interval', 3)           # Process every Nth frame
        self.declare_parameter('target_classes', '')            # Comma-separated: 'cup,bottle' (empty=all)

        self.model_path = self.get_parameter('model').value
        self.confidence = self.get_parameter('confidence').value
        self.device = self.get_parameter('device').value
        self.max_det = self.get_parameter('max_detections').value
        self.detect_interval = self.get_parameter('detect_interval').value

        target_str = self.get_parameter('target_classes').value
        self.target_classes = (
            [c.strip() for c in target_str.split(',') if c.strip()]
            if target_str else []
        )

        # --- YOLO model ---
        self.model = None
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'Loaded YOLO model: {self.model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
        else:
            self.get_logger().error(
                'ultralytics not installed! Run: pip install ultralytics'
            )

        # --- Publishers ---
        # JSON string with detection details
        self.detections_pub = self.create_publisher(
            String, '/detected_objects', 10
        )
        # Annotated image for debugging / web UI
        self.annotated_pub = self.create_publisher(
            Image, '/camera/image_annotated', 10
        )
        # RViz markers for detected objects
        self.marker_pub = self.create_publisher(
            MarkerArray, '/detected_objects_markers', 10
        )

        # --- Subscriber ---
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.frame_count = 0
        self.last_detections = []

        self.get_logger().info(
            f'Object detector ready — conf={self.confidence}, '
            f'interval={self.detect_interval}, '
            f'targets={self.target_classes or "all"}'
        )

    # ------------------------------------------------------------------ #
    #  Image callback                                                      #
    # ------------------------------------------------------------------ #
    def image_callback(self, msg):
        """Process incoming camera frame."""
        self.frame_count += 1

        # Only run detection every N frames to save CPU
        if self.frame_count % self.detect_interval != 0:
            return

        if self.model is None:
            return

        # Convert ROS Image → numpy array
        frame = self._ros_image_to_numpy(msg)
        if frame is None:
            return

        # Run YOLO inference
        try:
            results = self.model.predict(
                frame,
                conf=self.confidence,
                device=self.device,
                max_det=self.max_det,
                verbose=False
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference error: {e}')
            return

        # Parse results
        detections = self._parse_results(results, frame.shape)

        # Filter by target classes if specified
        if self.target_classes:
            detections = [
                d for d in detections
                if d['class_name'] in self.target_classes
            ]

        self.last_detections = detections

        # --- Publish detections as JSON ---
        det_msg = String()
        det_msg.data = json.dumps({
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.header.frame_id,
            'image_width': msg.width,
            'image_height': msg.height,
            'detections': detections
        })
        self.detections_pub.publish(det_msg)

        # --- Publish annotated image ---
        if OPENCV_AVAILABLE:
            annotated = self._draw_detections(frame.copy(), detections)
            self._publish_annotated(annotated, msg.header)

        if detections:
            names = [f"{d['class_name']}({d['confidence']:.0%})" for d in detections]
            self.get_logger().info(f'Detected: {", ".join(names)}')

    # ------------------------------------------------------------------ #
    #  YOLO result parsing                                                 #
    # ------------------------------------------------------------------ #
    def _parse_results(self, results, image_shape):
        """Parse YOLO results into a list of detection dicts."""
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
            cls_name = result.names.get(cls_id, f'class_{cls_id}')

            # Centre and normalised coordinates
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            bw = x2 - x1
            bh = y2 - y1

            detections.append({
                'class_id': cls_id,
                'class_name': cls_name,
                'confidence': round(conf, 3),
                'bbox_px': {
                    'x1': round(float(x1)),
                    'y1': round(float(y1)),
                    'x2': round(float(x2)),
                    'y2': round(float(y2)),
                },
                'center_px': {
                    'x': round(float(cx)),
                    'y': round(float(cy)),
                },
                'center_norm': {
                    'x': round(float(cx / w), 4),
                    'y': round(float(cy / h), 4),
                },
                'size_px': {
                    'w': round(float(bw)),
                    'h': round(float(bh)),
                },
            })

        # Sort by confidence descending
        detections.sort(key=lambda d: d['confidence'], reverse=True)
        return detections

    # ------------------------------------------------------------------ #
    #  Annotated image drawing                                             #
    # ------------------------------------------------------------------ #
    def _draw_detections(self, frame, detections):
        """Draw bounding boxes on the frame."""
        for det in detections:
            bb = det['bbox_px']
            label = f"{det['class_name']} {det['confidence']:.0%}"

            # Colour by class (simple hash)
            hue = (det['class_id'] * 47) % 180
            color_bgr = cv2.cvtColor(
                np.array([[[hue, 255, 200]]], dtype=np.uint8),
                cv2.COLOR_HSV2BGR
            )[0][0].tolist()

            cv2.rectangle(
                frame,
                (bb['x1'], bb['y1']),
                (bb['x2'], bb['y2']),
                color_bgr, 2
            )
            cv2.putText(
                frame, label,
                (bb['x1'], bb['y1'] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2
            )

        return frame

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #
    def _ros_image_to_numpy(self, msg):
        """Convert sensor_msgs/Image to numpy array (RGB)."""
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

    def _publish_annotated(self, frame_rgb, header):
        """Publish annotated image as ROS Image msg."""
        msg = Image()
        msg.header = header
        msg.height = frame_rgb.shape[0]
        msg.width = frame_rgb.shape[1]
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = frame_rgb.shape[1] * 3
        msg.data = frame_rgb.tobytes()
        self.annotated_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down object detector...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
