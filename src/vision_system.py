#!/usr/bin/env python3
"""
Vision System Module for EmoRobots Robot Arm
===========================================

This module handles computer vision tasks including face detection,
object detection, tracking, and coordinate mapping for robot control.

Author: EmoRobots Team
Date: July 25, 2025
"""

import cv2
import numpy as np
import logging
import time
import threading
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass

# Try to import advanced vision libraries
try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    mp = None

try:
    import tensorflow as tf
    TENSORFLOW_AVAILABLE = True
except ImportError:
    TENSORFLOW_AVAILABLE = False
    tf = None


@dataclass
class DetectedFace:
    """Data class for face detection results."""
    x: int
    y: int
    width: int
    height: int
    confidence: float
    center_x: int
    center_y: int
    
    @classmethod
    def from_bbox(cls, x: int, y: int, w: int, h: int, confidence: float = 1.0):
        """Create DetectedFace from bounding box coordinates."""
        return cls(x, y, w, h, confidence, x + w//2, y + h//2)


@dataclass
class DetectedObject:
    """Data class for object detection results."""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x, y, w, h
    center: Tuple[int, int]


class VisionSystem:
    """
    Computer vision system for face tracking and object detection.
    
    Handles camera input, face detection, object detection, and 
    coordinate mapping for robot control.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize vision system.
        
        Args:
            config: Vision system configuration
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # Camera configuration
        camera_config = config.get("camera", {})
        self.device_id = camera_config.get("device_id", 0)
        self.resolution = camera_config.get("resolution", {"width": 640, "height": 480})
        self.framerate = camera_config.get("framerate", 30)
        
        # Detection configuration
        face_config = config.get("face_detection", {})
        self.face_confidence_threshold = face_config.get("confidence_threshold", 0.7)
        self.max_faces = face_config.get("max_faces", 1)
        
        # Tracking configuration
        track_config = config.get("tracking", {})
        self.smoothing_factor = track_config.get("smoothing_factor", 0.8)
        self.prediction_frames = track_config.get("prediction_frames", 3)
        
        # Coordinate mapping configuration
        coord_config = config.get("coordinate_mapping", {})
        self.deadzone_radius = coord_config.get("deadzone_radius", 30)
        self.servo_gain = coord_config.get("servo_response_gain", 1.5)
        self.max_angular_velocity = coord_config.get("max_angular_velocity", 45)
        
        # Camera and detection objects
        self.camera = None
        self.face_detector = None
        self.object_detector = None
        
        # State tracking
        self.active = False
        self.last_frame = None
        self.last_faces = []
        self.face_history = []
        self.frame_lock = threading.Lock()
        
        # Coordinate mapping
        self.image_center = (
            self.resolution["width"] // 2,
            self.resolution["height"] // 2
        )
        
        self.logger.info("Vision system initialized")
    
    def initialize(self) -> bool:
        """
        Initialize camera and detection systems.
        
        Returns:
            bool: True if initialization successful
        """
        try:
            # Initialize camera
            if not self._initialize_camera():
                return False
            
            # Initialize face detection
            if not self._initialize_face_detection():
                return False
            
            # Initialize object detection (optional)
            self._initialize_object_detection()
            
            self.active = True
            self.logger.info("✅ Vision system initialized successfully")
            return True
        
        except Exception as e:
            self.logger.error(f"Failed to initialize vision system: {e}")
            return False
    
    def _initialize_camera(self) -> bool:
        """Initialize camera capture."""
        try:
            self.logger.info(f"🎥 Initializing camera {self.device_id}...")
            
            self.camera = cv2.VideoCapture(self.device_id)
            
            if not self.camera.isOpened():
                self.logger.error("Failed to open camera")
                return False
            
            # Set camera properties
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution["width"])
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution["height"])
            self.camera.set(cv2.CAP_PROP_FPS, self.framerate)
            
            # Test capture
            ret, frame = self.camera.read()
            if not ret or frame is None:
                self.logger.error("Failed to capture test frame")
                return False
            
            self.logger.info(f"✅ Camera initialized: {frame.shape[1]}x{frame.shape[0]}")
            return True
        
        except Exception as e:
            self.logger.error(f"Camera initialization failed: {e}")
            return False
    
    def _initialize_face_detection(self) -> bool:
        """Initialize face detection system."""
        face_config = self.config.get("face_detection", {})
        model_type = face_config.get("model_type", "opencv")
        
        try:
            if model_type == "mediapipe" and MEDIAPIPE_AVAILABLE:
                self.logger.info("🧠 Initializing MediaPipe face detection...")
                self.mp_face_detection = mp.solutions.face_detection
                self.face_detector = self.mp_face_detection.FaceDetection(
                    model_selection=0,
                    min_detection_confidence=self.face_confidence_threshold
                )
                self.logger.info("✅ MediaPipe face detection initialized")
                return True
            
            else:
                # Fallback to OpenCV Haar cascades
                self.logger.info("🔍 Initializing OpenCV face detection...")
                cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
                self.face_detector = cv2.CascadeClassifier(cascade_path)
                
                if self.face_detector.empty():
                    self.logger.error("Failed to load face cascade")
                    return False
                
                self.logger.info("✅ OpenCV face detection initialized")
                return True
        
        except Exception as e:
            self.logger.error(f"Face detection initialization failed: {e}")
            return False
    
    def _initialize_object_detection(self) -> bool:
        """Initialize object detection system (optional)."""
        try:
            object_config = self.config.get("object_detection", {})
            model_path = object_config.get("model_path")
            
            if not model_path or not TENSORFLOW_AVAILABLE:
                self.logger.info("⚠️  Object detection not available")
                return False
            
            # TODO: Implement TensorFlow Lite object detection
            self.logger.info("🎯 Object detection initialization skipped (not implemented)")
            return True
        
        except Exception as e:
            self.logger.warning(f"Object detection initialization failed: {e}")
            return False
    
    def capture_frame(self) -> Optional[np.ndarray]:
        """
        Capture a frame from the camera.
        
        Returns:
            Captured frame or None if failed
        """
        if not self.camera or not self.camera.isOpened():
            return None
        
        try:
            ret, frame = self.camera.read()
            if ret and frame is not None:
                with self.frame_lock:
                    self.last_frame = frame.copy()
                return frame
            return None
        
        except Exception as e:
            self.logger.error(f"Frame capture failed: {e}")
            return None
    
    def detect_faces(self) -> List[DetectedFace]:
        """
        Detect faces in the current frame.
        
        Returns:
            List of detected faces
        """
        frame = self.capture_frame()
        if frame is None:
            return []
        
        try:
            if hasattr(self, 'mp_face_detection') and self.face_detector:
                return self._detect_faces_mediapipe(frame)
            else:
                return self._detect_faces_opencv(frame)
        
        except Exception as e:
            self.logger.error(f"Face detection failed: {e}")
            return []
    
    def _detect_faces_mediapipe(self, frame: np.ndarray) -> List[DetectedFace]:
        """Detect faces using MediaPipe."""
        try:
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.face_detector.process(rgb_frame)
            
            faces = []
            if results.detections:
                h, w, _ = frame.shape
                
                for detection in results.detections[:self.max_faces]:
                    bbox = detection.location_data.relative_bounding_box
                    confidence = detection.score[0]
                    
                    if confidence >= self.face_confidence_threshold:
                        # Convert relative coordinates to absolute
                        x = int(bbox.xmin * w)
                        y = int(bbox.ymin * h)
                        width = int(bbox.width * w)
                        height = int(bbox.height * h)
                        
                        face = DetectedFace.from_bbox(x, y, width, height, confidence)
                        faces.append(face)
            
            self.last_faces = faces
            return faces
        
        except Exception as e:
            self.logger.error(f"MediaPipe face detection error: {e}")
            return []
    
    def _detect_faces_opencv(self, frame: np.ndarray) -> List[DetectedFace]:
        """Detect faces using OpenCV."""
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            faces_rects = self.face_detector.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30),
                flags=cv2.CASCADE_SCALE_IMAGE
            )
            
            faces = []
            for i, (x, y, w, h) in enumerate(faces_rects[:self.max_faces]):
                confidence = 1.0  # OpenCV doesn't provide confidence scores
                face = DetectedFace.from_bbox(x, y, w, h, confidence)
                faces.append(face)
            
            self.last_faces = faces
            return faces
        
        except Exception as e:
            self.logger.error(f"OpenCV face detection error: {e}")
            return []
    
    def face_to_servo_angles(self, face: DetectedFace) -> Optional[Dict[int, float]]:
        """
        Convert face position to servo angles for tracking.
        
        Args:
            face: Detected face object
            
        Returns:
            Dictionary of servo angles {servo_id: angle}
        """
        try:
            # Calculate offset from image center
            dx = face.center_x - self.image_center[0]
            dy = face.center_y - self.image_center[1]
            
            # Apply deadzone
            if abs(dx) < self.deadzone_radius and abs(dy) < self.deadzone_radius:
                return None  # Face is centered, no movement needed
            
            # Calculate servo angles
            # Servo 1: Vertical tilt (negative dy moves up)
            # Servo 2: Horizontal pan (positive dx moves right)
            
            # Normalize to [-1, 1] range
            dx_norm = dx / (self.image_center[0])
            dy_norm = dy / (self.image_center[1])
            
            # Apply gain and convert to angle adjustments
            h_angle_delta = dx_norm * self.servo_gain * self.max_angular_velocity
            v_angle_delta = -dy_norm * self.servo_gain * self.max_angular_velocity  # Negative for intuitive movement
            
            # Limit angular velocity
            h_angle_delta = max(-self.max_angular_velocity, min(self.max_angular_velocity, h_angle_delta))
            v_angle_delta = max(-self.max_angular_velocity, min(self.max_angular_velocity, v_angle_delta))
            
            # Return relative angle adjustments
            # Note: These would be added to current servo positions
            servo_angles = {
                1: v_angle_delta,  # Vertical servo
                2: h_angle_delta   # Horizontal servo
            }
            
            self.logger.debug(f"Face tracking: dx={dx}, dy={dy} → servo angles={servo_angles}")
            return servo_angles
        
        except Exception as e:
            self.logger.error(f"Face to servo angle conversion failed: {e}")
            return None
    
    def track_face_smooth(self, face: DetectedFace) -> Optional[Dict[int, float]]:
        """
        Track face with smoothing applied.
        
        Args:
            face: Current detected face
            
        Returns:
            Smoothed servo angles
        """
        # Add face to history
        self.face_history.append(face)
        
        # Keep only recent history
        max_history = max(5, self.prediction_frames)
        if len(self.face_history) > max_history:
            self.face_history = self.face_history[-max_history:]
        
        # Apply smoothing if we have enough history
        if len(self.face_history) < 2:
            return self.face_to_servo_angles(face)
        
        # Calculate smoothed position
        recent_faces = self.face_history[-3:]  # Use last 3 faces
        
        avg_x = sum(f.center_x for f in recent_faces) / len(recent_faces)
        avg_y = sum(f.center_y for f in recent_faces) / len(recent_faces)
        
        # Create smoothed face object
        smoothed_face = DetectedFace(
            face.x, face.y, face.width, face.height, face.confidence,
            int(avg_x), int(avg_y)
        )
        
        return self.face_to_servo_angles(smoothed_face)
    
    def get_frame_with_annotations(self) -> Optional[np.ndarray]:
        """
        Get current frame with detection annotations.
        
        Returns:
            Annotated frame or None
        """
        with self.frame_lock:
            if self.last_frame is None:
                return None
            
            frame = self.last_frame.copy()
        
        # Draw face detections
        for face in self.last_faces:
            # Draw bounding box
            cv2.rectangle(frame, (face.x, face.y), 
                         (face.x + face.width, face.y + face.height), 
                         (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(frame, (face.center_x, face.center_y), 5, (0, 255, 0), -1)
            
            # Draw confidence
            cv2.putText(frame, f"{face.confidence:.2f}", 
                       (face.x, face.y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, (0, 255, 0), 1)
        
        # Draw image center and deadzone
        cv2.circle(frame, self.image_center, 3, (255, 0, 0), -1)
        cv2.circle(frame, self.image_center, self.deadzone_radius, (255, 0, 0), 1)
        
        # Draw status info
        status_text = f"Faces: {len(self.last_faces)}"
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.7, (255, 255, 255), 2)
        
        return frame
    
    def is_active(self) -> bool:
        """Check if vision system is active."""
        return self.active and self.camera is not None and self.camera.isOpened()
    
    def start_preview(self, window_name: str = "Robot Vision") -> bool:
        """
        Start live preview window.
        
        Args:
            window_name: Name of preview window
            
        Returns:
            bool: True if preview started successfully
        """
        if not self.is_active():
            self.logger.error("Vision system not active")
            return False
        
        self.logger.info(f"Starting preview window: {window_name}")
        
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
            
            while True:
                frame = self.get_frame_with_annotations()
                if frame is not None:
                    cv2.imshow(window_name, frame)
                
                # Check for exit key
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 'q' or ESC
                    break
                elif key == ord(' '):  # Space to pause
                    cv2.waitKey(0)
            
            cv2.destroyWindow(window_name)
            self.logger.info("Preview window closed")
            return True
        
        except Exception as e:
            self.logger.error(f"Preview error: {e}")
            return False
    
    def calibrate_camera(self) -> Dict[str, float]:
        """
        Perform camera calibration (simplified).
        
        Returns:
            Calibration parameters
        """
        self.logger.info("Performing camera calibration...")
        
        # Capture several frames to check stability
        frame_count = 10
        frames = []
        
        for i in range(frame_count):
            frame = self.capture_frame()
            if frame is not None:
                frames.append(frame)
            time.sleep(0.1)
        
        if not frames:
            self.logger.error("No frames captured for calibration")
            return {}
        
        # Basic calibration metrics
        avg_brightness = np.mean([np.mean(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)) for f in frames])
        frame_variance = np.var([np.mean(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)) for f in frames])
        
        calibration_data = {
            "average_brightness": float(avg_brightness),
            "brightness_stability": float(frame_variance),
            "frame_count": len(frames),
            "resolution": f"{frames[0].shape[1]}x{frames[0].shape[0]}",
            "timestamp": time.time()
        }
        
        self.logger.info(f"Calibration complete: {calibration_data}")
        return calibration_data
    
    def stop(self):
        """Stop vision system and release resources."""
        self.logger.info("Stopping vision system...")
        
        self.active = False
        
        if self.camera:
            self.camera.release()
            self.camera = None
        
        cv2.destroyAllWindows()
        
        self.logger.info("✅ Vision system stopped")
    
    def __del__(self):
        """Cleanup on destruction."""
        self.stop()
