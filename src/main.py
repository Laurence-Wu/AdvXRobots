#!/usr/bin/env python3
"""
Main Application Entry Point for EmoRobots Robot Arm
===================================================

This module provides the main controller class that orchestrates all
robot arm subsystems including hardware interface, vision system,
display management, and sequence execution.

Author: EmoRobots Team
Date: July 25, 2025
"""

import os
import sys
import time
import json
import logging
import argparse
import threading
from typing import Dict, List, Optional, Tuple
from datetime import datetime

# Add current directory to path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import our modules
from hardware_interface import HardwareInterface
from vision_system import VisionSystem
from display_manager import DisplayManager
from kinematics import KinematicsEngine
from sequences import SequenceManager


class RobotArmController:
    """
    Main controller class for the robot arm system.
    
    Coordinates all subsystems and provides high-level control interface.
    """
    
    def __init__(self, config_path: str = None):
        """
        Initialize the robot arm controller.
        
        Args:
            config_path: Path to configuration file
        """
        self.config_path = config_path or self._get_default_config_path()
        self.config = self._load_config()
        
        # Setup logging
        self._setup_logging()
        self.logger = logging.getLogger(__name__)
        
        # Initialize subsystems
        self.hardware = None
        self.vision = None
        self.display = None
        self.kinematics = None
        self.sequences = None
        
        # Control state
        self.running = False
        self.face_tracking_active = False
        self.current_mode = "idle"
        
        # Threading
        self.main_thread = None
        self.vision_thread = None
        self.shutdown_event = threading.Event()
        
        self.logger.info("RobotArmController initialized")
    
    def _get_default_config_path(self) -> str:
        """Get default configuration file path."""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(current_dir, "..", "config", "arm_config.json")
    
    def _load_config(self) -> Dict:
        """Load configuration from file."""
        try:
            with open(self.config_path, 'r') as f:
                config = json.load(f)
            print(f"✅ Configuration loaded from {self.config_path}")
            return config
        except Exception as e:
            print(f"❌ Failed to load config: {e}")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict:
        """Get default configuration if file loading fails."""
        return {
            "hardware": {
                "servos": {"count": 6, "communication": {"port": None, "baudrate": 1000000}},
                "camera": {"device_id": 0, "resolution": {"width": 640, "height": 480}},
                "display": {"type": "LCD", "interface": "I2C"}
            },
            "vision": {
                "face_detection": {"confidence_threshold": 0.7, "max_faces": 1},
                "tracking": {"smoothing_factor": 0.8}
            },
            "behavior": {
                "face_tracking": {"enabled": True, "auto_start": False}
            },
            "system": {
                "logging": {"level": "INFO"},
                "performance": {"main_loop_frequency": 30}
            }
        }
    
    def _setup_logging(self):
        """Setup logging configuration."""
        log_config = self.config.get("system", {}).get("logging", {})
        log_level = getattr(logging, log_config.get("level", "INFO"))
        log_file = log_config.get("file_path", "logs/robot_arm.log")
        
        # Create logs directory if it doesn't exist
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler(sys.stdout)
            ]
        )
    
    def initialize_subsystems(self) -> bool:
        """
        Initialize all subsystems.
        
        Returns:
            bool: True if all subsystems initialized successfully
        """
        try:
            self.logger.info("Initializing subsystems...")
            
            # Initialize hardware interface
            self.hardware = HardwareInterface(self.config.get("hardware", {}))
            if not self.hardware.initialize():
                self.logger.error("Failed to initialize hardware interface")
                return False
            
            # Initialize vision system
            self.vision = VisionSystem(self.config.get("vision", {}))
            if not self.vision.initialize():
                self.logger.error("Failed to initialize vision system")
                return False
            
            # Initialize display manager
            self.display = DisplayManager(self.config.get("hardware", {}).get("display", {}))
            if not self.display.initialize():
                self.logger.warning("Display initialization failed, continuing without display")
            
            # Initialize kinematics engine
            self.kinematics = KinematicsEngine(self.config.get("kinematics", {}))
            
            # Initialize sequence manager
            self.sequences = SequenceManager(
                self.hardware, 
                self.config.get("sequences", {})
            )
            
            self.logger.info("✅ All subsystems initialized successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to initialize subsystems: {e}")
            return False
    
    def start(self, mode: str = "idle") -> bool:
        """
        Start the robot arm controller.
        
        Args:
            mode: Operating mode ("idle", "face_tracking", "sequence")
            
        Returns:
            bool: True if started successfully
        """
        if self.running:
            self.logger.warning("Controller already running")
            return True
        
        self.logger.info(f"Starting robot arm controller in {mode} mode")
        
        if not self.initialize_subsystems():
            return False
        
        self.running = True
        self.current_mode = mode
        
        # Start main control loop
        self.main_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.main_thread.start()
        
        # Start mode-specific operations
        if mode == "face_tracking":
            self.start_face_tracking()
        elif mode.startswith("sequence"):
            sequence_name = mode.split(":")[-1] if ":" in mode else "wave"
            self.execute_sequence(sequence_name)
        
        self.logger.info("✅ Robot arm controller started")
        return True
    
    def stop(self):
        """Stop the robot arm controller."""
        self.logger.info("Stopping robot arm controller...")
        
        self.running = False
        self.face_tracking_active = False
        self.shutdown_event.set()
        
        # Stop vision processing
        if self.vision:
            self.vision.stop()
        
        # Return servos to safe positions
        if self.hardware:
            self.hardware.emergency_stop()
        
        # Wait for threads to finish
        if self.main_thread and self.main_thread.is_alive():
            self.main_thread.join(timeout=2.0)
        
        if self.vision_thread and self.vision_thread.is_alive():
            self.vision_thread.join(timeout=2.0)
        
        self.logger.info("✅ Robot arm controller stopped")
    
    def _main_loop(self):
        """Main control loop running in separate thread."""
        loop_frequency = self.config.get("system", {}).get("performance", {}).get("main_loop_frequency", 30)
        loop_interval = 1.0 / loop_frequency
        
        self.logger.info(f"Main loop started at {loop_frequency} Hz")
        
        while self.running and not self.shutdown_event.is_set():
            try:
                start_time = time.time()
                
                # Update display
                if self.display:
                    self._update_display()
                
                # Check system health
                self._check_system_health()
                
                # Sleep to maintain loop frequency
                elapsed = time.time() - start_time
                sleep_time = max(0, loop_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
            except Exception as e:
                self.logger.error(f"Error in main loop: {e}")
                time.sleep(0.1)
        
        self.logger.info("Main loop finished")
    
    def start_face_tracking(self) -> bool:
        """
        Start face tracking mode.
        
        Returns:
            bool: True if started successfully
        """
        if not self.vision or not self.hardware:
            self.logger.error("Vision system or hardware not initialized")
            return False
        
        if self.face_tracking_active:
            self.logger.warning("Face tracking already active")
            return True
        
        self.logger.info("Starting face tracking...")
        self.face_tracking_active = True
        
        # Start vision processing thread
        self.vision_thread = threading.Thread(target=self._face_tracking_loop, daemon=True)
        self.vision_thread.start()
        
        # Display status
        if self.display:
            self.display.show_status("Face Tracking", "ACTIVE")
        
        return True
    
    def stop_face_tracking(self):
        """Stop face tracking mode."""
        self.logger.info("Stopping face tracking...")
        self.face_tracking_active = False
        
        if self.display:
            self.display.show_status("Face Tracking", "STOPPED")
    
    def _face_tracking_loop(self):
        """Face tracking processing loop."""
        self.logger.info("Face tracking loop started")
        
        tracking_config = self.config.get("vision", {}).get("tracking", {})
        vision_fps = self.config.get("system", {}).get("performance", {}).get("vision_processing_fps", 15)
        frame_interval = 1.0 / vision_fps
        
        while self.face_tracking_active and not self.shutdown_event.is_set():
            try:
                start_time = time.time()
                
                # Get face detection results
                faces = self.vision.detect_faces()
                
                if faces:
                    # Track the primary face (first detected)
                    face = faces[0]
                    
                    # Convert face position to servo angles
                    servo_angles = self.vision.face_to_servo_angles(face)
                    
                    if servo_angles:
                        # Apply smoothing
                        smoothed_angles = self._apply_tracking_smoothing(servo_angles)
                        
                        # Send commands to servos
                        self.hardware.move_tracking_servos(smoothed_angles)
                        
                        self.logger.debug(f"Tracking face at {smoothed_angles}")
                else:
                    # No face detected - implement search behavior if configured
                    search_enabled = self.config.get("behavior", {}).get("face_tracking", {}).get("search_on_lost", False)
                    if search_enabled:
                        self._execute_search_pattern()
                
                # Maintain frame rate
                elapsed = time.time() - start_time
                sleep_time = max(0, frame_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
            except Exception as e:
                self.logger.error(f"Error in face tracking loop: {e}")
                time.sleep(0.1)
        
        self.logger.info("Face tracking loop finished")
    
    def _apply_tracking_smoothing(self, servo_angles: Dict[int, float]) -> Dict[int, float]:
        """
        Apply smoothing to servo angles for stable tracking.
        
        Args:
            servo_angles: Raw servo angles from vision system
            
        Returns:
            Dict[int, float]: Smoothed servo angles
        """
        smoothing_factor = self.config.get("vision", {}).get("tracking", {}).get("smoothing_factor", 0.8)
        
        # For now, just return the input angles
        # TODO: Implement proper smoothing with previous values
        return servo_angles
    
    def _execute_search_pattern(self):
        """Execute search pattern when face is lost."""
        # TODO: Implement search pattern
        pass
    
    def execute_sequence(self, sequence_name: str) -> bool:
        """
        Execute a predefined movement sequence.
        
        Args:
            sequence_name: Name of sequence to execute
            
        Returns:
            bool: True if sequence executed successfully
        """
        if not self.sequences:
            self.logger.error("Sequence manager not initialized")
            return False
        
        self.logger.info(f"Executing sequence: {sequence_name}")
        
        # Display status
        if self.display:
            self.display.show_status("Sequence", sequence_name)
        
        return self.sequences.execute_sequence(sequence_name)
    
    def move_to_position(self, x: float, y: float, z: float) -> bool:
        """
        Move end effector to specified position using inverse kinematics.
        
        Args:
            x, y, z: Target position coordinates
            
        Returns:
            bool: True if movement successful
        """
        if not self.kinematics or not self.hardware:
            self.logger.error("Kinematics or hardware not initialized")
            return False
        
        # Calculate joint angles using inverse kinematics
        joint_angles = self.kinematics.inverse_kinematics(x, y, z)
        
        if joint_angles is None:
            self.logger.error(f"Unable to reach position ({x}, {y}, {z})")
            return False
        
        # Execute movement
        return self.hardware.move_to_joint_angles(joint_angles)
    
    def _update_display(self):
        """Update display with current status."""
        if not self.display:
            return
        
        status_info = {
            "mode": self.current_mode,
            "tracking": "ON" if self.face_tracking_active else "OFF",
            "hardware": "OK" if self.hardware and self.hardware.is_connected() else "ERROR",
            "timestamp": datetime.now().strftime("%H:%M:%S")
        }
        
        self.display.update_status(status_info)
    
    def _check_system_health(self):
        """Check system health and handle any issues."""
        # Check hardware connection
        if self.hardware and not self.hardware.is_connected():
            self.logger.warning("Hardware connection lost")
            # Try to reconnect
            self.hardware.reconnect()
        
        # Check vision system
        if self.vision and not self.vision.is_active():
            self.logger.warning("Vision system inactive")
    
    def get_status(self) -> Dict:
        """
        Get current system status.
        
        Returns:
            Dict: System status information
        """
        return {
            "running": self.running,
            "mode": self.current_mode,
            "face_tracking": self.face_tracking_active,
            "hardware_connected": self.hardware.is_connected() if self.hardware else False,
            "vision_active": self.vision.is_active() if self.vision else False,
            "display_connected": self.display.is_connected() if self.display else False,
            "timestamp": datetime.now().isoformat()
        }


def main():
    """Main function for command-line interface."""
    parser = argparse.ArgumentParser(description="EmoRobots Robot Arm Controller")
    parser.add_argument("--mode", choices=["idle", "face_tracking", "sequence"], 
                       default="idle", help="Operating mode")
    parser.add_argument("--sequence", default="wave", 
                       help="Sequence name (when mode=sequence)")
    parser.add_argument("--config", help="Configuration file path")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    parser.add_argument("--log-level", choices=["DEBUG", "INFO", "WARNING", "ERROR"], 
                       default="INFO", help="Logging level")
    
    args = parser.parse_args()
    
    # Create controller
    controller = RobotArmController(config_path=args.config)
    
    # Override log level if specified
    if args.debug or args.log_level == "DEBUG":
        logging.getLogger().setLevel(logging.DEBUG)
    
    try:
        print("🤖 EmoRobots Robot Arm Controller")
        print("=" * 50)
        
        # Determine mode
        mode = args.mode
        if mode == "sequence":
            mode = f"sequence:{args.sequence}"
        
        # Start controller
        if controller.start(mode):
            print(f"✅ Robot arm started in {args.mode} mode")
            
            if args.mode == "face_tracking":
                print("🎯 Face tracking active - press Ctrl+C to stop")
            elif args.mode == "sequence":
                print(f"🎭 Executing sequence '{args.sequence}' - press Ctrl+C to stop")
            else:
                print("💤 Idle mode - press Ctrl+C to stop")
            
            # Keep running until interrupted
            try:
                while controller.running:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n🛑 Shutdown requested...")
        else:
            print("❌ Failed to start robot arm controller")
            return 1
    
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1
    
    finally:
        controller.stop()
        print("👋 Robot arm controller stopped")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
