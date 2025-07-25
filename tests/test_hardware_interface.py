#!/usr/bin/env python3
"""
Test Hardware Interface Module
============================

Unit tests for the hardware interface module.
"""

import unittest
import sys
import os
from unittest.mock import Mock, patch

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from hardware_interface import HardwareInterface, ServoStatus


class TestHardwareInterface(unittest.TestCase):
    """Test cases for HardwareInterface class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.config = {
            "servos": {
                "count": 3,
                "communication": {
                    "port": None,  # Force mock mode
                    "baudrate": 1000000,
                    "timeout": 0.1
                },
                "servo_configs": {
                    "0": {
                        "id": 0,
                        "name": "base",
                        "angle_range": [-180, 180],
                        "initial_position": 0,
                        "power_mode": "active"
                    },
                    "1": {
                        "id": 1,
                        "name": "shoulder",
                        "angle_range": [-120, 120],
                        "initial_position": -90,
                        "power_mode": "active"
                    },
                    "2": {
                        "id": 2,
                        "name": "elbow",
                        "angle_range": [-90, 90],
                        "initial_position": 0,
                        "power_mode": "active"
                    }
                },
                "motion_params": {
                    "default_speed": 500,
                    "angle_tolerance": 2.0,
                    "power_settings": {
                        "active_power": 8000,
                        "holding_power": 4000,
                        "zero_power": 0
                    }
                }
            }
        }
    
    def test_initialization(self):
        """Test hardware interface initialization."""
        hardware = HardwareInterface(self.config)
        self.assertIsNotNone(hardware)
        self.assertFalse(hardware.use_hardware)  # Should use mock mode
    
    def test_mock_initialization(self):
        """Test mock hardware initialization."""
        hardware = HardwareInterface(self.config)
        success = hardware.initialize()
        self.assertTrue(success)
        self.assertTrue(hardware.is_connected())
    
    def test_servo_movement(self):
        """Test servo movement commands."""
        hardware = HardwareInterface(self.config)
        hardware.initialize()
        
        # Test single servo movement
        success = hardware.move_servo(1, 45.0, duration=1000)
        self.assertTrue(success)
        
        # Test multiple servo movement
        servo_angles = {0: 30.0, 1: -45.0, 2: 60.0}
        success = hardware.move_multiple_servos(servo_angles)
        self.assertTrue(success)
    
    def test_servo_status(self):
        """Test servo status queries."""
        hardware = HardwareInterface(self.config)
        hardware.initialize()
        
        # Test individual servo status
        status = hardware.get_servo_status(1)
        self.assertIsInstance(status, ServoStatus)
        self.assertEqual(status.id, 1)
        self.assertTrue(status.online)
        
        # Test all servo statuses
        all_statuses = hardware.get_all_servo_statuses()
        self.assertIsInstance(all_statuses, dict)
        self.assertGreater(len(all_statuses), 0)
    
    def test_emergency_stop(self):
        """Test emergency stop functionality."""
        hardware = HardwareInterface(self.config)
        hardware.initialize()
        
        # Should not raise exception
        hardware.emergency_stop()
    
    def test_angle_limits(self):
        """Test angle limiting functionality."""
        hardware = HardwareInterface(self.config)
        hardware.initialize()
        
        # Test angle within limits
        success = hardware.move_servo(1, 100.0)  # Within [-120, 120]
        self.assertTrue(success)
        
        # Test angle clamping (should still succeed but be clamped)
        success = hardware.move_servo(1, 150.0)  # Outside range, should be clamped
        self.assertTrue(success)


if __name__ == '__main__':
    unittest.main()
