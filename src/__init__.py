"""
EmoRobots - My Robot Arm Package
==============================

A modular robotic arm control system with face tracking capabilities.

Modules:
    - main: Main application entry point
    - hardware_interface: Hardware abstraction layer
    - vision_system: Computer vision and face tracking
    - display_manager: LCD display management
    - kinematics: Robotic arm kinematics calculations
    - sequences: Pre-programmed movement sequences
"""

__version__ = "1.0.0"
__author__ = "EmoRobots Team"
__email__ = "team@emorobots.org"

# Import main classes for easy access
from .main import RobotArmController
from .hardware_interface import HardwareInterface
from .vision_system import VisionSystem
from .display_manager import DisplayManager
from .kinematics import KinematicsEngine
from .sequences import SequenceManager

__all__ = [
    'RobotArmController',
    'HardwareInterface', 
    'VisionSystem',
    'DisplayManager',
    'KinematicsEngine',
    'SequenceManager'
]
