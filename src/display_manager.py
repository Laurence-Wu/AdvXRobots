#!/usr/bin/env python3
"""
Display Manager Module for EmoRobots Robot Arm
==============================================

This module manages LCD displays and status information display
for the robot arm system.

Author: EmoRobots Team
Date: July 25, 2025
"""

import time
import logging
from typing import Dict, List, Optional, Any
from datetime import datetime


class MockDisplay:
    """Mock display for testing without hardware."""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__ + ".MockDisplay")
        self.logger.info("🖥️  Initializing Mock Display")
        self.current_status = {}
    
    def initialize(self) -> bool:
        """Initialize mock display."""
        self.logger.info("✅ Mock display initialized")
        return True
    
    def clear(self):
        """Clear display."""
        self.logger.debug("🖥️  Mock: Display cleared")
    
    def show_text(self, text: str, line: int = 0):
        """Show text on display."""
        self.logger.debug(f"🖥️  Mock: Line {line}: {text}")
    
    def show_status(self, title: str, value: str):
        """Show status information."""
        self.current_status[title] = value
        self.logger.info(f"🖥️  Mock Status: {title} = {value}")
    
    def is_connected(self) -> bool:
        """Check if display is connected."""
        return True


class DisplayManager:
    """
    Display management system for robot arm status and information.
    
    Handles LCD displays, status updates, and user interface elements.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize display manager.
        
        Args:
            config: Display configuration dictionary
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # Display configuration
        self.display_type = config.get("type", "LCD")
        self.interface = config.get("interface", "I2C")
        self.address = config.get("address", "0x3C")
        self.resolution = config.get("resolution", {"width": 128, "height": 64})
        self.refresh_rate = config.get("refresh_rate", 10)
        
        # Display hardware
        self.display = None
        self.connected = False
        self.use_hardware = False  # Set to True when real hardware is available
        
        # Status tracking
        self.current_status = {}
        self.last_update = 0
        self.update_interval = 1.0 / self.refresh_rate
        
        self.logger.info(f"Display manager initialized ({self.display_type})")
    
    def initialize(self) -> bool:
        """
        Initialize display hardware.
        
        Returns:
            bool: True if initialization successful
        """
        try:
            if self.use_hardware:
                return self._initialize_hardware()
            else:
                return self._initialize_mock()
        
        except Exception as e:
            self.logger.error(f"Failed to initialize display: {e}")
            return False
    
    def _initialize_hardware(self) -> bool:
        """Initialize real display hardware."""
        try:
            # TODO: Implement actual hardware display initialization
            # This would typically involve I2C/SPI setup for OLED/LCD displays
            self.logger.warning("Hardware display not implemented - using mock")
            return self._initialize_mock()
        
        except Exception as e:
            self.logger.error(f"Hardware display initialization failed: {e}")
            return self._initialize_mock()
    
    def _initialize_mock(self) -> bool:
        """Initialize mock display for testing."""
        try:
            self.display = MockDisplay()
            self.connected = self.display.initialize()
            self.use_hardware = False
            
            if self.connected:
                self.logger.info("✅ Mock display initialized")
                self._show_startup_screen()
                return True
            else:
                return False
        
        except Exception as e:
            self.logger.error(f"Mock display initialization failed: {e}")
            return False
    
    def _show_startup_screen(self):
        """Show startup/welcome screen."""
        try:
            self.display.clear()
            self.display.show_text("EmoRobots", 0)
            self.display.show_text("Robot Arm v1.0", 1)
            self.display.show_text("Initializing...", 2)
            time.sleep(1.0)
        except Exception as e:
            self.logger.error(f"Error showing startup screen: {e}")
    
    def is_connected(self) -> bool:
        """Check if display is connected."""
        return self.connected and self.display is not None
    
    def show_status(self, title: str, value: str):
        """
        Show status information on display.
        
        Args:
            title: Status title/label
            value: Status value
        """
        if not self.is_connected():
            return
        
        try:
            self.current_status[title] = value
            self.display.show_status(title, value)
            self.logger.debug(f"Status updated: {title} = {value}")
        
        except Exception as e:
            self.logger.error(f"Error showing status: {e}")
    
    def update_status(self, status_dict: Dict[str, Any]):
        """
        Update multiple status items.
        
        Args:
            status_dict: Dictionary of status items
        """
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_update < self.update_interval:
            return
        
        if not self.is_connected():
            return
        
        try:
            self.display.clear()
            
            # Show main status items
            line = 0
            
            # Mode and tracking status
            mode = status_dict.get("mode", "Unknown")
            tracking = status_dict.get("tracking", "OFF")
            self.display.show_text(f"Mode: {mode}", line)
            line += 1
            
            self.display.show_text(f"Track: {tracking}", line)
            line += 1
            
            # Hardware status
            hardware_status = status_dict.get("hardware", "Unknown")
            self.display.show_text(f"HW: {hardware_status}", line)
            line += 1
            
            # Timestamp
            timestamp = status_dict.get("timestamp", "")
            if isinstance(timestamp, str) and len(timestamp) > 8:
                timestamp = timestamp[-8:]  # Show only time part
            self.display.show_text(f"Time: {timestamp}", line)
            
            self.current_status.update(status_dict)
            self.last_update = current_time
            
        except Exception as e:
            self.logger.error(f"Error updating status: {e}")
    
    def show_message(self, message: str, duration: float = 2.0):
        """
        Show a temporary message.
        
        Args:
            message: Message to display
            duration: Display duration in seconds
        """
        if not self.is_connected():
            return
        
        try:
            self.display.clear()
            self.display.show_text(message, 1)  # Center message
            time.sleep(duration)
            
            # Restore previous status
            self._restore_status_display()
            
        except Exception as e:
            self.logger.error(f"Error showing message: {e}")
    
    def show_error(self, error_message: str):
        """
        Show error message.
        
        Args:
            error_message: Error message to display
        """
        if not self.is_connected():
            return
        
        try:
            self.display.clear()
            self.display.show_text("ERROR!", 0)
            
            # Split long error messages
            if len(error_message) > 20:
                self.display.show_text(error_message[:20], 1)
                if len(error_message) > 20:
                    self.display.show_text(error_message[20:40], 2)
            else:
                self.display.show_text(error_message, 1)
            
            self.logger.info(f"Error displayed: {error_message}")
            
        except Exception as e:
            self.logger.error(f"Error showing error message: {e}")
    
    def show_servo_status(self, servo_statuses: Dict[int, Any]):
        """
        Show servo status information.
        
        Args:
            servo_statuses: Dictionary of servo status data
        """
        if not self.is_connected():
            return
        
        try:
            self.display.clear()
            self.display.show_text("Servo Status", 0)
            
            line = 1
            for servo_id, status in list(servo_statuses.items())[:3]:  # Show first 3 servos
                if hasattr(status, 'online') and status.online:
                    angle_str = f"{status.angle:.0f}°" if hasattr(status, 'angle') else "??°"
                    self.display.show_text(f"S{servo_id}: {angle_str}", line)
                else:
                    self.display.show_text(f"S{servo_id}: OFF", line)
                line += 1
                
                if line > 3:  # Display limitation
                    break
            
        except Exception as e:
            self.logger.error(f"Error showing servo status: {e}")
    
    def show_face_tracking_info(self, faces_detected: int, tracking_active: bool):
        """
        Show face tracking information.
        
        Args:
            faces_detected: Number of faces detected
            tracking_active: Whether tracking is active
        """
        if not self.is_connected():
            return
        
        try:
            self.display.clear()
            self.display.show_text("Face Tracking", 0)
            
            status = "ACTIVE" if tracking_active else "INACTIVE"
            self.display.show_text(f"Status: {status}", 1)
            
            self.display.show_text(f"Faces: {faces_detected}", 2)
            
            # Show timestamp
            current_time = datetime.now().strftime("%H:%M:%S")
            self.display.show_text(current_time, 3)
            
        except Exception as e:
            self.logger.error(f"Error showing face tracking info: {e}")
    
    def show_sequence_info(self, sequence_name: str, step: int, total_steps: int):
        """
        Show sequence execution information.
        
        Args:
            sequence_name: Name of executing sequence
            step: Current step number
            total_steps: Total number of steps
        """
        if not self.is_connected():
            return
        
        try:
            self.display.clear()
            self.display.show_text("Sequence", 0)
            
            # Truncate long sequence names
            display_name = sequence_name[:12] if len(sequence_name) > 12 else sequence_name
            self.display.show_text(display_name, 1)
            
            progress = f"Step {step}/{total_steps}"
            self.display.show_text(progress, 2)
            
            # Show progress bar (simplified)
            if total_steps > 0:
                progress_percent = int((step / total_steps) * 100)
                self.display.show_text(f"Progress: {progress_percent}%", 3)
            
        except Exception as e:
            self.logger.error(f"Error showing sequence info: {e}")
    
    def show_system_info(self, info: Dict[str, Any]):
        """
        Show system information.
        
        Args:
            info: System information dictionary
        """
        if not self.is_connected():
            return
        
        try:
            self.display.clear()
            self.display.show_text("System Info", 0)
            
            # Show key system metrics
            line = 1
            if "cpu_usage" in info:
                self.display.show_text(f"CPU: {info['cpu_usage']:.1f}%", line)
                line += 1
            
            if "memory_usage" in info:
                self.display.show_text(f"RAM: {info['memory_usage']:.1f}%", line)
                line += 1
            
            if "temperature" in info:
                self.display.show_text(f"Temp: {info['temperature']:.1f}°C", line)
                line += 1
            
        except Exception as e:
            self.logger.error(f"Error showing system info: {e}")
    
    def _restore_status_display(self):
        """Restore the previous status display."""
        if self.current_status:
            self.update_status(self.current_status)
    
    def clear_display(self):
        """Clear the display."""
        if not self.is_connected():
            return
        
        try:
            self.display.clear()
        except Exception as e:
            self.logger.error(f"Error clearing display: {e}")
    
    def set_brightness(self, brightness: float):
        """
        Set display brightness.
        
        Args:
            brightness: Brightness level (0.0 to 1.0)
        """
        if not self.is_connected():
            return
        
        try:
            # TODO: Implement brightness control for real hardware
            self.logger.debug(f"Display brightness set to {brightness:.1%}")
        except Exception as e:
            self.logger.error(f"Error setting brightness: {e}")
    
    def get_display_info(self) -> Dict[str, Any]:
        """
        Get display information.
        
        Returns:
            Dictionary with display information
        """
        return {
            "type": self.display_type,
            "interface": self.interface,
            "resolution": self.resolution,
            "connected": self.connected,
            "use_hardware": self.use_hardware,
            "refresh_rate": self.refresh_rate,
            "last_update": self.last_update
        }
    
    def shutdown(self):
        """Shutdown display system."""
        try:
            if self.is_connected():
                self.display.clear()
                self.display.show_text("Shutting down...", 1)
                time.sleep(1.0)
                self.display.clear()
            
            self.connected = False
            self.display = None
            
            self.logger.info("✅ Display system shutdown")
        
        except Exception as e:
            self.logger.error(f"Error during display shutdown: {e}")
    
    def __del__(self):
        """Cleanup on destruction."""
        self.shutdown()
