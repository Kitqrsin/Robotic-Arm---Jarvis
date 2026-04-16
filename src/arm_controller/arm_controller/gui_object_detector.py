#!/usr/bin/env python3
"""
Lightweight **face & object** detection for the TKinter GUI camera panel.

Face backends (mode='face'):
  Primary:  OpenCV DNN YuNet face detector (FaceDetectorYN)
  Fallback: Haar cascades

Object backend (mode='cup'):
  YOLOv8n ONNX via cv2.dnn — detects cups, bottles, wine-glasses
  (COCO classes 39/40/41).

Detection runs in a background thread every N-th frame to keep the
Raspberry Pi responsive.
"""

import threading
import os
import numpy as np

# ---------- Optional imports ----------
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    from PIL import Image as PILImage, ImageDraw, ImageFont
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

# Colour palette per class family
_FACE_COLOR = (0, 200, 255)   # cyan
_CUP_COLOR  = (50, 255, 50)   # green
_BOTTLE_COLOR = (255, 160, 0) # orange
_RED_OBJ_COLOR = (255, 50, 50)  # red

# HSV ranges for red object detection (red wraps around hue 0/180)
_RED_HSV_LO1 = (0,   100, 100)
_RED_HSV_HI1 = (10,  255, 255)
_RED_HSV_LO2 = (170, 100, 100)
_RED_HSV_HI2 = (180, 255, 255)
_RED_MIN_AREA = 800  # minimum contour area in pixels

# COCO class index → friendly name & colour
_CUP_CLASSES = {
    39: ('bottle',     _BOTTLE_COLOR),
    40: ('wine glass', _CUP_COLOR),
    41: ('cup',        _CUP_COLOR),
}

# YuNet model paths to search (first match wins)
_YUNET_MODEL_CANDIDATES = [
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 '..', '..', '..', '..', 'face_detection_yunet_2023mar.onnx'),
    '/workspace/face_detection_yunet_2023mar.onnx',
    os.path.expanduser('~/face_detection_yunet_2023mar.onnx'),
]

# YOLOv8n ONNX model paths to search
_YOLO_MODEL_CANDIDATES = [
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 '..', '..', '..', '..', 'yolov8n.onnx'),
    '/workspace/yolov8n.onnx',
    os.path.expanduser('~/yolov8n.onnx'),
]


class GUIObjectDetector:
    """In-process face, cup & red-object detector for the TKinter camera panel.

    Modes:
      'face' — YuNet or Haar cascade face detection (default)
      'cup'  — YOLOv8n ONNX object detection for cups/bottles/glasses
      'red'  — HSV color filtering for red objects
    """

    def __init__(
        self,
        cascade: str = 'haarcascade_frontalface_default.xml',
        confidence: float = 0.6,
        detect_every_n: int = 2,
        min_face_size: int = 50,
        scale_factor: float = 1.15,
        min_neighbours: int = 5,
        logger=None,
        **_kwargs,
    ):
        self.detect_every_n = detect_every_n
        self.min_face_size = min_face_size
        self.scale_factor = scale_factor
        self.min_neighbours = min_neighbours
        self._conf_threshold = confidence
        self._logger = logger

        self.enabled = False
        self._mode = 'face'             # 'face', 'cup', or 'red'
        self._frame_counter = 0
        self._latest_detections: list[dict] = []
        self._det_lock = threading.Lock()
        self._detection_running = False

        # --- Face: YuNet DNN ---
        self._yunet = None
        self.face_cascade = None
        self._face_backend = None       # 'yunet' or 'haar'

        if CV2_AVAILABLE and hasattr(cv2, 'FaceDetectorYN'):
            model_path = self._find_model(_YUNET_MODEL_CANDIDATES)
            if model_path:
                try:
                    self._yunet = cv2.FaceDetectorYN.create(
                        model_path, '', (640, 480),
                        self._conf_threshold, 0.3, 5000,
                        cv2.dnn.DNN_BACKEND_OPENCV,
                        cv2.dnn.DNN_TARGET_CPU,
                    )
                    self._face_backend = 'yunet'
                    self._log('info', f'Face detector: YuNet DNN ({model_path})')
                except Exception as e:
                    self._log('warn', f'YuNet init failed ({e}), falling back to Haar')

        if self._yunet is None and CV2_AVAILABLE:
            if os.path.isfile(cascade):
                cascade_path = cascade
            else:
                cv2_data = getattr(cv2, 'data', None)
                data_dir = cv2_data.haarcascades if cv2_data else ''
                cascade_path = os.path.join(data_dir, cascade)
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            if self.face_cascade.empty():
                self._log('error', f'Failed to load cascade from {cascade_path}')
                self.face_cascade = None
            else:
                self._face_backend = 'haar'
                self._log('info', f'Face detector: Haar cascade ({cascade_path})')

        # --- Cup: YOLOv8n ONNX via cv2.dnn ---
        self._yolo_net = None
        self._yolo_input_size = 640
        self._yolo_conf = 0.45
        self._yolo_nms = 0.45

        if CV2_AVAILABLE:
            yolo_path = self._find_model(_YOLO_MODEL_CANDIDATES)
            if yolo_path:
                try:
                    self._yolo_net = cv2.dnn.readNetFromONNX(yolo_path)
                    self._yolo_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                    self._yolo_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                    self._log('info', f'Cup detector: YOLOv8n ONNX ({yolo_path})')
                except Exception as e:
                    self._log('warn', f'YOLOv8n ONNX load failed: {e}')

        # Combined backend flag
        self._backend = self._face_backend  # initially face mode

    @staticmethod
    def _find_model(candidates):
        for p in candidates:
            rp = os.path.realpath(p)
            if os.path.isfile(rp):
                return rp
        return None

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #
    def toggle(self, on: bool):
        """Enable or disable detection."""
        self.enabled = on

    def set_mode(self, mode: str):
        """Switch detection mode: 'face', 'cup', or 'red'."""
        if mode not in ('face', 'cup', 'red'):
            return
        self._mode = mode
        # Clear stale detections when switching mode
        with self._det_lock:
            self._latest_detections = []

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def cup_detection_available(self) -> bool:
        return self._yolo_net is not None

    @property
    def detections(self) -> list[dict]:
        """Return the latest detection list (thread-safe copy)."""
        with self._det_lock:
            return list(self._latest_detections)

    def process_frame(self, frame_rgb: np.ndarray) -> np.ndarray:
        """Run detection (if due) and draw overlays on *frame_rgb*."""
        if not self.enabled:
            return frame_rgb

        # Check that the active mode has a backend
        if self._mode == 'face' and self._face_backend is None:
            return frame_rgb
        if self._mode == 'cup' and self._yolo_net is None:
            return frame_rgb
        if self._mode == 'red' and not CV2_AVAILABLE:
            return frame_rgb

        self._frame_counter += 1

        n = self.detect_every_n if self._mode == 'face' else max(self.detect_every_n, 4)
        if self._frame_counter % n == 0:
            if not self._detection_running:
                det_frame = frame_rgb.copy()
                self._detection_running = True
                threading.Thread(
                    target=self._bg_detect, args=(det_frame,), daemon=True
                ).start()

        return self._draw_overlay(frame_rgb)

    # ------------------------------------------------------------------ #
    #  Internal — detection                                                #
    # ------------------------------------------------------------------ #
    def _bg_detect(self, frame_rgb: np.ndarray):
        try:
            self._run_detection(frame_rgb)
        finally:
            self._detection_running = False

    def _run_detection(self, frame_rgb: np.ndarray):
        """Run detection and store results based on current mode."""
        try:
            if self._mode == 'red':
                self._run_red_hsv(frame_rgb)
            elif self._mode == 'cup' and self._yolo_net is not None:
                self._run_yolo(frame_rgb)
            elif self._yunet is not None:
                self._run_yunet(frame_rgb)
            elif self.face_cascade is not None:
                self._run_haar(frame_rgb)
        except Exception as e:
            self._log('error', f'Detection error: {e}')

    def _run_yunet(self, frame_rgb: np.ndarray):
        """YuNet DNN face detection."""
        h, w = frame_rgb.shape[:2]
        self._yunet.setInputSize((w, h))

        # YuNet expects BGR
        bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        retval, faces = self._yunet.detect(bgr)

        detections: list[dict] = []
        if faces is not None:
            for face in faces:
                # face: [x, y, w, h, ..., score]
                fx, fy, fw, fh = int(face[0]), int(face[1]), int(face[2]), int(face[3])
                score = float(face[14]) if face.shape[0] > 14 else float(face[-1])
                if fw < self.min_face_size or fh < self.min_face_size:
                    continue
                detections.append({
                    'x1': fx, 'y1': fy,
                    'x2': fx + fw, 'y2': fy + fh,
                    'confidence': score,
                    'class_id': 0,
                    'class_name': 'face',
                })

        detections.sort(
            key=lambda d: (d['x2'] - d['x1']) * (d['y2'] - d['y1']),
            reverse=True,
        )
        with self._det_lock:
            self._latest_detections = detections

    def _run_haar(self, frame_rgb: np.ndarray):
        """Haar cascade face detection (fallback)."""
        if CV2_AVAILABLE:
            gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        else:
            gray = np.dot(frame_rgb[..., :3], [0.299, 0.587, 0.114]).astype(np.uint8)

        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbours,
            minSize=(self.min_face_size, self.min_face_size),
        )

        detections: list[dict] = []
        h, w = frame_rgb.shape[:2]
        for (fx, fy, fw, fh) in faces:
            area_frac = (fw * fh) / (w * h) if (w * h) > 0 else 0
            detections.append({
                'x1': int(fx), 'y1': int(fy),
                'x2': int(fx + fw), 'y2': int(fy + fh),
                'confidence': min(1.0, area_frac / 0.08),
                'class_id': 0,
                'class_name': 'face',
            })

        detections.sort(
            key=lambda d: (d['x2'] - d['x1']) * (d['y2'] - d['y1']),
            reverse=True,
        )
        with self._det_lock:
            self._latest_detections = detections

    def _run_yolo(self, frame_rgb: np.ndarray):
        """YOLOv8n ONNX object detection for cups/bottles/glasses."""
        h, w = frame_rgb.shape[:2]
        sz = self._yolo_input_size

        # Pre-process: letterbox to sz×sz
        bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        blob = cv2.dnn.blobFromImage(bgr, 1 / 255.0, (sz, sz),
                                     swapRB=True, crop=False)
        self._yolo_net.setInput(blob)
        out = self._yolo_net.forward()  # shape (1, 84, 8400)

        # Transpose to (8400, 84)
        preds = out[0].T  # (8400, 84)
        # columns: cx, cy, w, h, class_scores[80]

        # Scale factors (letterbox → original)
        sx = w / sz
        sy = h / sz

        boxes = []
        scores = []
        class_ids = []
        for row in preds:
            class_scores = row[4:]
            max_score = float(np.max(class_scores))
            if max_score < self._yolo_conf:
                continue
            cls_id = int(np.argmax(class_scores))
            if cls_id not in _CUP_CLASSES:
                continue
            cx, cy, bw, bh = row[0] * sx, row[1] * sy, row[2] * sx, row[3] * sy
            x1 = int(cx - bw / 2)
            y1 = int(cy - bh / 2)
            x2 = int(cx + bw / 2)
            y2 = int(cy + bh / 2)
            boxes.append([x1, y1, x2 - x1, y2 - y1])
            scores.append(max_score)
            class_ids.append(cls_id)

        detections: list[dict] = []
        if boxes:
            idxs = cv2.dnn.NMSBoxes(boxes, scores, self._yolo_conf, self._yolo_nms)
            for i in idxs.flatten():
                bx, by, bw, bh = boxes[i]
                cls_id = class_ids[i]
                name, _ = _CUP_CLASSES[cls_id]
                detections.append({
                    'x1': max(0, bx), 'y1': max(0, by),
                    'x2': min(w, bx + bw), 'y2': min(h, by + bh),
                    'confidence': scores[i],
                    'class_id': cls_id,
                    'class_name': name,
                })
            detections.sort(
                key=lambda d: (d['x2'] - d['x1']) * (d['y2'] - d['y1']),
                reverse=True,
            )

        with self._det_lock:
            self._latest_detections = detections

    # ------------------------------------------------------------------ #
    #  Internal — drawing                                                  #
    # ------------------------------------------------------------------ #
    def _draw_overlay(self, frame_rgb: np.ndarray) -> np.ndarray:
        with self._det_lock:
            dets = list(self._latest_detections)
        if not dets:
            return frame_rgb

        if PIL_AVAILABLE:
            return self._draw_with_pil(frame_rgb, dets)
        else:
            return self._draw_with_numpy(frame_rgb, dets)

    def _run_red_hsv(self, frame_rgb: np.ndarray):
        """Detect red objects via HSV colour filtering."""
        hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
        mask1 = cv2.inRange(hsv, np.array(_RED_HSV_LO1), np.array(_RED_HSV_HI1))
        mask2 = cv2.inRange(hsv, np.array(_RED_HSV_LO2), np.array(_RED_HSV_HI2))
        mask = mask1 | mask2

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        h, w = frame_rgb.shape[:2]
        detections: list[dict] = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < _RED_MIN_AREA:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            detections.append({
                'x1': x, 'y1': y,
                'x2': x + bw, 'y2': y + bh,
                'confidence': min(1.0, area / (w * h * 0.05)),
                'class_id': 99,
                'class_name': 'red_object',
            })

        detections.sort(
            key=lambda d: (d['x2'] - d['x1']) * (d['y2'] - d['y1']),
            reverse=True,
        )
        with self._det_lock:
            self._latest_detections = detections

    @staticmethod
    def _det_color(det):
        """Pick colour for a detection."""
        cls_id = det.get('class_id', 0)
        if cls_id == 99:
            return _RED_OBJ_COLOR
        if cls_id in _CUP_CLASSES:
            return _CUP_CLASSES[cls_id][1]
        return _FACE_COLOR

    @staticmethod
    def _draw_with_pil(frame_rgb: np.ndarray, dets: list[dict]) -> np.ndarray:
        img = PILImage.fromarray(frame_rgb)
        draw = ImageDraw.Draw(img)
        try:
            font = ImageFont.truetype(
                '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 13
            )
        except Exception:
            font = ImageFont.load_default()

        for det in dets:
            cls_id = det.get('class_id', 0)
            cls_id = det.get('class_id', 0)
            if cls_id == 99:
                color = _RED_OBJ_COLOR
            elif cls_id in _CUP_CLASSES:
                color = _CUP_CLASSES[cls_id][1]
            else:
                color = _FACE_COLOR
            x1, y1, x2, y2 = det['x1'], det['y1'], det['x2'], det['y2']
            label = f"{det['class_name']} {det['confidence']:.0%}"

            # Bounding box (2px)
            for w in range(2):
                draw.rectangle([x1 - w, y1 - w, x2 + w, y2 + w], outline=color)

            # Label background
            bbox = draw.textbbox((0, 0), label, font=font)
            tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
            label_y = max(y1 - th - 6, 0)
            draw.rectangle(
                [x1, label_y, x1 + tw + 6, label_y + th + 4], fill=color)
            draw.text(
                (x1 + 3, label_y + 1), label,
                fill=(255, 255, 255), font=font)

        return np.array(img)

    @staticmethod
    def _draw_with_numpy(frame_rgb: np.ndarray, dets: list[dict]) -> np.ndarray:
        for det in dets:
            cls_id = det.get('class_id', 0)
            color = _RED_OBJ_COLOR if cls_id == 99 else (
                _CUP_CLASSES[cls_id][1] if cls_id in _CUP_CLASSES else _FACE_COLOR)
            x1, y1, x2, y2 = det['x1'], det['y1'], det['x2'], det['y2']
            frame_rgb[y1:y1+2, x1:x2] = color
            frame_rgb[y2-2:y2, x1:x2] = color
            frame_rgb[y1:y2, x1:x1+2] = color
            frame_rgb[y1:y2, x2-2:x2] = color
        return frame_rgb

    def _log(self, level, msg):
        if self._logger:
            getattr(self._logger, level, self._logger.info)(msg)
