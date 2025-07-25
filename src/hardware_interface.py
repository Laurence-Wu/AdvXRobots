#!/usr/bin/env python3
"""
Hardware Interface Module for EmoRobots Robot Arm
================================================

This module provides a hardware abstraction layer for servo motors,
sensors, and other hardware components. It handles communication
protocols, error handling, and hardware-specific implementations.

Author: EmoRobots Team
Date: July 25, 2025
"""

import os
import sys
import time
import logging
import serial
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from threading import Lock

# Try to import servo SDK
try:
    # Add SDK path
    sdk_path = os.path.join(os.path.dirname(__file__), "vendor", "seeed_arm_sdk")
    if os.path.exists(sdk_path):
        sys.path.append(sdk_path)
    
    import fashionstar_uart_sdk as servo_sdk
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    servo_sdk = None


@dataclass
class ServoStatus:
    """Data class for servo status information."""
    id: int
    online: bool
    angle: float
    voltage: float
    current: float
    temperature: float
    error_code: int = 0


class MockServoManager:
    """Mock servo manager for testing without hardware."""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__ + ".MockServoManager")
        self.logger.info("🎮 Initializing Mock Servo Manager")
        
        # Mock servo states
        self.mock_servos = {0: True, 1: True, 2: True, 3: True, 4: True, 5: False}
        self.mock_angles = {0: 0.0, 1: -90.0, 2: -90.0, 3: 0.0, 4: 0.0}
        self.mock_voltages = {i: 8.4 for i in range(6)}
        self.mock_currents = {i: 0.5 for i in range(6)}
        self.mock_temperatures = {i: 35.0 for i in range(6)}
    
    def ping(self, servo_id: int) -> bool:
        """Check if servo is online."""
        return self.mock_servos.get(servo_id, False)
    
    def query_servo_angle(self, servo_id: int) -> Optional[float]:
        """Get current servo angle."""
        return self.mock_angles.get(servo_id)
    
    def set_servo_angle(self, servo_id: int, angle: float, interval: int = 500, 
                       power: int = 8000, **kwargs) -> bool:
        """Set servo angle."""
        if servo_id in self.mock_angles:
            self.mock_angles[servo_id] = angle
            self.logger.debug(f"🎮 Mock: Servo {servo_id} → {angle}° (time: {interval}ms, power: {power}mW)")
            time.sleep(interval / 10000.0)  # Simulate movement time
            return True
        return False
    
    def set_damping(self, servo_id: int, power: int = 0) -> bool:
        """Set servo damping (power mode)."""
        if servo_id in self.mock_servos:
            self.logger.debug(f"🎮 Mock: Servo {servo_id} damping → {power}")
            return True
        return False
    
    def query_voltage(self, servo_id: int) -> Optional[float]:
        """Get servo voltage."""
        return self.mock_voltages.get(servo_id)
    
    def query_current(self, servo_id: int) -> Optional[float]:
        """Get servo current."""
        return self.mock_currents.get(servo_id)
    
    def query_temperature(self, servo_id: int) -> Optional[float]:
        """Get servo temperature."""
        return self.mock_temperatures.get(servo_id)


class HardwareInterface:
    """
    Hardware abstraction layer for robot arm control.
    
    Provides unified interface for servo motors, sensors, and other hardware.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize hardware interface.
        
        Args:
            config: Hardware configuration dictionary
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # Communication setup
        self.serial_config = config.get("servos", {}).get("communication", {})
        self.port = self.serial_config.get("port")
        self.baudrate = self.serial_config.get("baudrate", 1000000)
        self.timeout = self.serial_config.get("timeout", 0.1)
        
        # Servo configuration
        self.servo_configs = config.get("servos", {}).get("servo_configs", {})
        self.motion_params = config.get("servos", {}).get("motion_params", {})
        
        # Hardware components
        self.uart = None
        self.servo_manager = None
        self.connected = False
        self.use_hardware = SDK_AVAILABLE and bool(self.port)
        
        # Thread safety
        self.command_lock = Lock()
        
        # Status tracking
        self.servo_statuses = {}
        self.last_positions = {}
        
        self.logger.info(f"Hardware interface initialized (Hardware: {self.use_hardware})")
    
    def initialize(self) -> bool:
        """
        Initialize hardware connection and components.
        
        Returns:
            bool: True if initialization successful
        """
        try:
            if self.use_hardware:
                return self._initialize_hardware()
            else:
                return self._initialize_mock()
        
        except Exception as e:
            self.logger.error(f"Failed to initialize hardware: {e}")
            return False
    
    def _initialize_hardware(self) -> bool:
        """Initialize real hardware connection."""
        try:
            self.logger.info(f"🔌 Connecting to {self.port} @ {self.baudrate} baud...")
            
            self.uart = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=1,
                bytesize=8
            )
            
            self.servo_manager = servo_sdk.UartServoManager(self.uart)
            
            # Test connection by pinging servos
            connected_servos = 0
            for servo_id in range(6):  # Test first 6 servos
                if self.servo_manager.ping(servo_id):
                    connected_servos += 1
                    self.logger.debug(f"✅ Servo {servo_id}: Connected")
                else:
                    self.logger.debug(f"❌ Servo {servo_id}: Not found")
            
            if connected_servos > 0:
                self.connected = True
                self.logger.info(f"✅ Hardware initialized ({connected_servos} servos found)")
                
                # Initialize servo positions
                self._initialize_servo_positions()
                return True
            else:
                self.logger.warning("No servos detected - switching to mock mode")
                return self._initialize_mock()
        
        except Exception as e:
            self.logger.error(f"Hardware initialization failed: {e}")
            self.logger.info("Falling back to mock mode")
            return self._initialize_mock()
    
    def _initialize_mock(self) -> bool:
        """Initialize mock hardware for testing."""
        try:
            self.servo_manager = MockServoManager()
            self.connected = True
            self.use_hardware = False
            
            self.logger.info("✅ Mock hardware initialized")
            
            # Initialize mock servo positions
            self._initialize_servo_positions()
            return True
        
        except Exception as e:
            self.logger.error(f"Mock initialization failed: {e}")
            return False
    
    def _initialize_servo_positions(self):
        """Initialize servos to their configured starting positions."""
        self.logger.info("Initializing servo positions...")
        
        for servo_id_str, servo_config in self.servo_configs.items():
            servo_id = int(servo_id_str)
            initial_position = servo_config.get("initial_position", 0)
            power_mode = servo_config.get("power_mode", "active")
            
            try:
                # Set initial position
                if power_mode == "active":
                    success = self.servo_manager.set_servo_angle(
                        servo_id, initial_position, interval=1000, power=8000
                    )
                    if success:
                        self.last_positions[servo_id] = initial_position
                        self.logger.debug(f"✅ Servo {servo_id}: {initial_position}°")
                    else:
                        self.logger.warning(f"⚠️  Servo {servo_id}: Failed to set position")
                
                elif power_mode == "zero_power":
                    self.servo_manager.set_damping(servo_id, 0)
                    self.logger.debug(f"🔋 Servo {servo_id}: Zero power mode")
                
                # Update status
                self._update_servo_status(servo_id)
            
            except Exception as e:
                self.logger.error(f"Failed to initialize servo {servo_id}: {e}")
        
        # Wait for movements to complete
        time.sleep(1.5)
        self.logger.info("Servo initialization complete")
    
    def is_connected(self) -> bool:
        """Check if hardware is connected."""
        return self.connected
    
    def reconnect(self) -> bool:
        """Attempt to reconnect to hardware."""
        self.logger.info("Attempting to reconnect...")
        self.disconnect()
        return self.initialize()
    
    def disconnect(self):
        """Disconnect from hardware."""
        try:
            if self.uart and hasattr(self.uart, 'close'):
                self.uart.close()
            self.connected = False
            self.logger.info("🔌 Hardware disconnected")
        except Exception as e:
            self.logger.error(f"Disconnect error: {e}")
    
    def move_servo(self, servo_id: int, angle: float, duration: int = None, 
                   power: int = None) -> bool:
        """
        Move a single servo to specified angle.
        
        Args:
            servo_id: Servo ID
            angle: Target angle in degrees
            duration: Movement duration in milliseconds
            power: Servo power (0-8000)
            
        Returns:
            bool: True if movement successful
        """
        if not self.connected or not self.servo_manager:
            self.logger.error("Hardware not connected")
            return False
        
        # Get servo configuration
        servo_config = self.servo_configs.get(str(servo_id))
        if not servo_config:
            self.logger.error(f"No configuration for servo {servo_id}")
            return False
        
        # Apply angle limits
        angle_range = servo_config.get("angle_range", [-180, 180])
        clamped_angle = max(angle_range[0], min(angle_range[1], angle))
        
        if clamped_angle != angle:
            self.logger.warning(f"Servo {servo_id}: Angle {angle}° clamped to {clamped_angle}°")
        
        # Use default parameters if not specified
        if duration is None:
            duration = self.motion_params.get("default_speed", 500)
        if power is None:
            power = self.motion_params.get("power_settings", {}).get("active_power", 8000)
        
        try:
            with self.command_lock:
                success = self.servo_manager.set_servo_angle(
                    servo_id, clamped_angle, interval=duration, power=power
                )
                
                if success:
                    self.last_positions[servo_id] = clamped_angle
                    self.logger.debug(f"Servo {servo_id}: {clamped_angle}° (time: {duration}ms)")
                    return True
                else:
                    self.logger.error(f"Failed to move servo {servo_id}")
                    return False
        
        except Exception as e:
            self.logger.error(f"Error moving servo {servo_id}: {e}")
            return False
    
    def move_multiple_servos(self, servo_angles: Dict[int, float], 
                           duration: int = None, synchronous: bool = True) -> bool:
        """
        Move multiple servos simultaneously.
        
        Args:
            servo_angles: Dictionary of {servo_id: angle}
            duration: Movement duration in milliseconds
            synchronous: Whether to wait for completion
            
        Returns:
            bool: True if all movements successful
        """
        if not servo_angles:
            return True
        
        success_count = 0
        
        for servo_id, angle in servo_angles.items():
            if self.move_servo(servo_id, angle, duration):
                success_count += 1
        
        if synchronous and duration:
            # Wait for movements to complete
            time.sleep(duration / 1000.0 + 0.1)
        
        return success_count == len(servo_angles)
    
    def move_tracking_servos(self, angles: Dict[int, float]) -> bool:
        """
        Move face tracking servos (typically servos 1 & 2).
        
        Args:
            angles: Dictionary of servo angles
            
        Returns:
            bool: True if movement successful
        """
        # Filter to only tracking servos (1 & 2)
        tracking_angles = {
            servo_id: angle 
            for servo_id, angle in angles.items() 
            if servo_id in [1, 2]
        }
        
        return self.move_multiple_servos(
            tracking_angles, 
            duration=200,  # Fast tracking movements
            synchronous=False
        )
    
    def move_to_joint_angles(self, joint_angles: List[float]) -> bool:
        """
        Move to specified joint angles.
        
        Args:
            joint_angles: List of joint angles in degrees
            
        Returns:
            bool: True if movement successful
        """
        if len(joint_angles) > len(self.servo_configs):
            self.logger.error("Too many joint angles provided")
            return False
        
        servo_angles = {}
        for i, angle in enumerate(joint_angles):
            if str(i) in self.servo_configs:
                servo_angles[i] = angle
        
        return self.move_multiple_servos(servo_angles, duration=1000)
    
    def get_servo_status(self, servo_id: int) -> Optional[ServoStatus]:
        """
        Get detailed status of a servo.
        
        Args:
            servo_id: Servo ID
            
        Returns:
            ServoStatus object or None if failed
        """
        if not self.connected or not self.servo_manager:
            return None
        
        try:
            online = self.servo_manager.ping(servo_id)
            if not online:
                return ServoStatus(servo_id, False, 0.0, 0.0, 0.0, 0.0)
            
            angle = self.servo_manager.query_servo_angle(servo_id) or 0.0
            voltage = self.servo_manager.query_voltage(servo_id) or 0.0
            current = self.servo_manager.query_current(servo_id) or 0.0
            temperature = self.servo_manager.query_temperature(servo_id) or 0.0
            
            return ServoStatus(servo_id, True, angle, voltage, current, temperature)
        
        except Exception as e:
            self.logger.error(f"Error getting servo {servo_id} status: {e}")
            return None
    
    def _update_servo_status(self, servo_id: int):
        """Update cached servo status."""
        status = self.get_servo_status(servo_id)
        if status:
            self.servo_statuses[servo_id] = status
    
    def get_all_servo_statuses(self) -> Dict[int, ServoStatus]:
        """
        Get status of all configured servos.
        
        Returns:
            Dictionary of servo statuses
        """
        statuses = {}
        
        for servo_id_str in self.servo_configs.keys():
            servo_id = int(servo_id_str)
            status = self.get_servo_status(servo_id)
            if status:
                statuses[servo_id] = status
        
        return statuses
    
    def emergency_stop(self):
        """Emergency stop - return all servos to safe positions."""
        self.logger.warning("🚨 EMERGENCY STOP activated")
        
        try:
            # Move to safe positions quickly
            safe_positions = {}
            for servo_id_str, servo_config in self.servo_configs.items():
                servo_id = int(servo_id_str)
                safe_position = servo_config.get("initial_position", 0)
                safe_positions[servo_id] = safe_position
            
            # Execute emergency positioning
            self.move_multiple_servos(safe_positions, duration=500, synchronous=True)
            
            # Set zero power on all servos
            for servo_id_str in self.servo_configs.keys():
                servo_id = int(servo_id_str)
                try:
                    self.servo_manager.set_damping(servo_id, 0)
                except Exception as e:
                    self.logger.error(f"Failed to set zero power on servo {servo_id}: {e}")
            
            self.logger.info("✅ Emergency stop completed")
        
        except Exception as e:
            self.logger.error(f"Error during emergency stop: {e}")
    
    def set_servo_power_mode(self, servo_id: int, power: int) -> bool:
        """
        Set servo power/damping mode.
        
        Args:
            servo_id: Servo ID
            power: Power level (0 = zero power, 8000 = full power)
            
        Returns:
            bool: True if successful
        """
        try:
            success = self.servo_manager.set_damping(servo_id, power)
            if success:
                self.logger.debug(f"Servo {servo_id} power set to {power}")
            return success
        except Exception as e:
            self.logger.error(f"Error setting servo {servo_id} power: {e}")
            return False
    
    def calibrate_servo(self, servo_id: int) -> bool:
        """
        Calibrate a servo by moving through its range.
        
        Args:
            servo_id: Servo ID to calibrate
            
        Returns:
            bool: True if calibration successful
        """
        servo_config = self.servo_configs.get(str(servo_id))
        if not servo_config:
            self.logger.error(f"No configuration for servo {servo_id}")
            return False
        
        angle_range = servo_config.get("angle_range", [-90, 90])
        initial_position = servo_config.get("initial_position", 0)
        
        self.logger.info(f"Calibrating servo {servo_id}...")
        
        try:
            # Move to minimum
            self.move_servo(servo_id, angle_range[0], duration=1000)
            time.sleep(1.2)
            
            # Move to maximum
            self.move_servo(servo_id, angle_range[1], duration=2000)
            time.sleep(2.2)
            
            # Return to initial position
            self.move_servo(servo_id, initial_position, duration=1000)
            time.sleep(1.2)
            
            self.logger.info(f"✅ Servo {servo_id} calibration complete")
            return True
        
        except Exception as e:
            self.logger.error(f"Servo {servo_id} calibration failed: {e}")
            return False
    
    def get_current_positions(self) -> Dict[int, float]:
        """
        Get current positions of all servos.
        
        Returns:
            Dictionary of current servo positions
        """
        positions = {}
        
        for servo_id_str in self.servo_configs.keys():
            servo_id = int(servo_id_str)
            try:
                angle = self.servo_manager.query_servo_angle(servo_id)
                if angle is not None:
                    positions[servo_id] = angle
            except Exception as e:
                self.logger.error(f"Error getting position for servo {servo_id}: {e}")
        
        return positions
    
    def __del__(self):
        """Cleanup on destruction."""
        self.disconnect()
