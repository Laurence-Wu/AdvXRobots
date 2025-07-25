#!/usr/bin/env python3
"""
Kinematics Engine Module for EmoRobots Robot Arm
===============================================

This module handles forward and inverse kinematics calculations
for the robot arm, including workspace validation and motion planning.

Author: EmoRobots Team  
Date: July 25, 2025
"""

import math
import numpy as np
import logging
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass


@dataclass
class JointConfiguration:
    """Data class for joint configuration."""
    angles: List[float]  # Joint angles in degrees
    valid: bool = True
    reachable: bool = True
    error_message: str = ""


@dataclass  
class CartesianPosition:
    """Data class for Cartesian position."""
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


class KinematicsEngine:
    """
    Kinematics engine for robot arm calculations.
    
    Handles forward kinematics, inverse kinematics, workspace validation,
    and trajectory planning for the robot arm.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize kinematics engine.
        
        Args:
            config: Kinematics configuration dictionary
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        
        # Arm parameters
        arm_params = config.get("arm_parameters", {})
        self.base_height = arm_params.get("base_height", 50.0)
        self.link_lengths = arm_params.get("link_lengths", [100.0, 150.0, 120.0])
        self.joint_offsets = arm_params.get("joint_offsets", [0.0, 90.0, 0.0])
        self.end_effector_length = arm_params.get("end_effector_length", 80.0)
        
        # Workspace constraints
        workspace = config.get("workspace", {})
        self.x_range = workspace.get("x_range", [-200, 200])
        self.y_range = workspace.get("y_range", [-200, 200])
        self.z_range = workspace.get("z_range", [0, 300])
        
        # Motion constraints
        constraints = config.get("constraints", {})
        self.max_reach = constraints.get("max_reach", 370.0)
        self.min_reach = constraints.get("min_reach", 50.0)
        self.max_joint_velocity = constraints.get("max_joint_velocity", 60.0)
        self.collision_avoidance = constraints.get("collision_avoidance", True)
        
        # Derived parameters
        self.total_reach = sum(self.link_lengths) + self.end_effector_length
        self.dof = len(self.link_lengths)  # Degrees of freedom
        
        self.logger.info(f"Kinematics engine initialized: {self.dof} DOF, reach: {self.total_reach:.1f}mm")
    
    def forward_kinematics(self, joint_angles: List[float]) -> Optional[CartesianPosition]:
        """
        Calculate forward kinematics from joint angles to end effector position.
        
        Args:
            joint_angles: List of joint angles in degrees
            
        Returns:
            CartesianPosition object or None if calculation failed
        """
        try:
            if len(joint_angles) < self.dof:
                self.logger.error(f"Insufficient joint angles: got {len(joint_angles)}, need {self.dof}")
                return None
            
            # Convert to radians and apply offsets
            angles_rad = [math.radians(angle + offset) 
                         for angle, offset in zip(joint_angles[:self.dof], self.joint_offsets)]
            
            # Forward kinematics calculation for typical robot arm
            # This is a simplified 3-DOF calculation
            
            # Joint 0: Base rotation (around Z-axis)
            # Joint 1: Shoulder elevation (around Y-axis)  
            # Joint 2: Elbow flexion (around Y-axis)
            
            base_angle = angles_rad[0] if len(angles_rad) > 0 else 0.0
            shoulder_angle = angles_rad[1] if len(angles_rad) > 1 else 0.0
            elbow_angle = angles_rad[2] if len(angles_rad) > 2 else 0.0
            
            # Calculate end effector position
            # Link 1 (shoulder to elbow)
            x1 = self.link_lengths[0] * math.cos(shoulder_angle)
            z1 = self.link_lengths[0] * math.sin(shoulder_angle)
            
            # Link 2 (elbow to wrist)
            x2 = self.link_lengths[1] * math.cos(shoulder_angle + elbow_angle)
            z2 = self.link_lengths[1] * math.sin(shoulder_angle + elbow_angle)
            
            # End effector
            x3 = self.end_effector_length * math.cos(shoulder_angle + elbow_angle)
            z3 = self.end_effector_length * math.sin(shoulder_angle + elbow_angle)
            
            # Total position in arm coordinate system
            arm_x = x1 + x2 + x3
            arm_z = z1 + z2 + z3 + self.base_height
            
            # Apply base rotation to get world coordinates
            world_x = arm_x * math.cos(base_angle)
            world_y = arm_x * math.sin(base_angle)
            world_z = arm_z
            
            # Calculate orientation (simplified)
            end_angle = shoulder_angle + elbow_angle
            pitch = math.degrees(end_angle)
            yaw = math.degrees(base_angle)
            
            position = CartesianPosition(
                x=world_x,
                y=world_y, 
                z=world_z,
                roll=0.0,
                pitch=pitch,
                yaw=yaw
            )
            
            self.logger.debug(f"Forward kinematics: joints={joint_angles} → pos=({world_x:.1f}, {world_y:.1f}, {world_z:.1f})")
            return position
            
        except Exception as e:
            self.logger.error(f"Forward kinematics calculation failed: {e}")
            return None
    
    def inverse_kinematics(self, x: float, y: float, z: float, 
                          orientation: Optional[Tuple[float, float, float]] = None) -> Optional[List[float]]:
        """
        Calculate inverse kinematics from end effector position to joint angles.
        
        Args:
            x, y, z: Target position coordinates
            orientation: Optional (roll, pitch, yaw) in degrees
            
        Returns:
            List of joint angles in degrees or None if unreachable
        """
        try:
            # Check if position is within workspace
            if not self.is_position_reachable(x, y, z):
                self.logger.warning(f"Position ({x:.1f}, {y:.1f}, {z:.1f}) is not reachable")
                return None
            
            # Inverse kinematics calculation for 3-DOF arm
            
            # Base joint (Joint 0): rotation to point towards target
            base_angle = math.atan2(y, x)
            
            # Distance from base to target in XY plane
            r = math.sqrt(x*x + y*y)
            
            # Adjust Z for base height
            z_adjusted = z - self.base_height
            
            # Distance from shoulder to target
            target_distance = math.sqrt(r*r + z_adjusted*z_adjusted)
            
            # Check if target is reachable with arm links
            link1_length = self.link_lengths[0]
            link2_length = self.link_lengths[1] + self.end_effector_length
            
            if target_distance > (link1_length + link2_length):
                self.logger.warning(f"Target distance {target_distance:.1f} exceeds max reach {link1_length + link2_length:.1f}")
                return None
            
            if target_distance < abs(link1_length - link2_length):
                self.logger.warning(f"Target distance {target_distance:.1f} is too close")
                return None
            
            # Elbow joint (Joint 2): use cosine rule
            cos_elbow = (link1_length*link1_length + link2_length*link2_length - target_distance*target_distance) / (2 * link1_length * link2_length)
            
            # Clamp to valid range to avoid numerical errors
            cos_elbow = max(-1.0, min(1.0, cos_elbow))
            elbow_angle = math.acos(cos_elbow)
            
            # Choose elbow-up configuration (negative angle)
            elbow_angle = -elbow_angle
            
            # Shoulder joint (Joint 1): angle to reach target
            angle_to_target = math.atan2(z_adjusted, r)
            angle_correction = math.asin((link2_length * math.sin(abs(elbow_angle))) / target_distance)
            
            shoulder_angle = angle_to_target - angle_correction
            
            # Convert to degrees and apply joint offsets
            joint_angles = [
                math.degrees(base_angle) - self.joint_offsets[0],
                math.degrees(shoulder_angle) - self.joint_offsets[1], 
                math.degrees(elbow_angle) - self.joint_offsets[2]
            ]
            
            # Validate joint limits (if configured)
            if not self._validate_joint_limits(joint_angles):
                self.logger.warning("Calculated joint angles exceed limits")
                return None
            
            self.logger.debug(f"Inverse kinematics: pos=({x:.1f}, {y:.1f}, {z:.1f}) → joints={joint_angles}")
            return joint_angles
            
        except Exception as e:
            self.logger.error(f"Inverse kinematics calculation failed: {e}")
            return None
    
    def is_position_reachable(self, x: float, y: float, z: float) -> bool:
        """
        Check if a position is within the robot's workspace.
        
        Args:
            x, y, z: Position coordinates
            
        Returns:
            bool: True if position is reachable
        """
        try:
            # Check workspace bounds
            if not (self.x_range[0] <= x <= self.x_range[1]):
                return False
            if not (self.y_range[0] <= y <= self.y_range[1]):
                return False
            if not (self.z_range[0] <= z <= self.z_range[1]):
                return False
            
            # Check distance from base
            distance_from_base = math.sqrt(x*x + y*y + (z - self.base_height)**2)
            
            if distance_from_base > self.max_reach:
                return False
            if distance_from_base < self.min_reach:
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"Reachability check failed: {e}")
            return False
    
    def _validate_joint_limits(self, joint_angles: List[float]) -> bool:
        """
        Validate joint angles against configured limits.
        
        Args:
            joint_angles: List of joint angles in degrees
            
        Returns:
            bool: True if all angles are within limits
        """
        # TODO: Implement joint limit checking based on configuration
        # For now, use reasonable defaults
        
        default_limits = [
            (-180, 180),  # Base rotation
            (-120, 120),  # Shoulder
            (-150, 150),  # Elbow
        ]
        
        for i, angle in enumerate(joint_angles[:len(default_limits)]):
            min_limit, max_limit = default_limits[i]
            if not (min_limit <= angle <= max_limit):
                self.logger.warning(f"Joint {i} angle {angle:.1f}° exceeds limits [{min_limit}, {max_limit}]")
                return False
        
        return True
    
    def plan_trajectory(self, start_angles: List[float], end_angles: List[float], 
                       duration: float, steps: int = 50) -> List[List[float]]:
        """
        Plan a smooth trajectory between two joint configurations.
        
        Args:
            start_angles: Starting joint angles in degrees
            end_angles: Target joint angles in degrees
            duration: Movement duration in seconds
            steps: Number of interpolation steps
            
        Returns:
            List of joint angle configurations for trajectory
        """
        try:
            if len(start_angles) != len(end_angles):
                self.logger.error("Start and end angle lists must have same length")
                return []
            
            trajectory = []
            
            for i in range(steps + 1):
                # Linear interpolation with smooth acceleration/deceleration
                t = i / steps
                
                # Apply smooth S-curve interpolation
                smooth_t = self._smooth_interpolation(t)
                
                # Interpolate joint angles
                interpolated_angles = []
                for start, end in zip(start_angles, end_angles):
                    angle = start + (end - start) * smooth_t
                    interpolated_angles.append(angle)
                
                trajectory.append(interpolated_angles)
            
            self.logger.debug(f"Trajectory planned: {len(trajectory)} steps over {duration:.1f}s")
            return trajectory
            
        except Exception as e:
            self.logger.error(f"Trajectory planning failed: {e}")
            return []
    
    def _smooth_interpolation(self, t: float) -> float:
        """
        Apply smooth interpolation curve (S-curve) for natural motion.
        
        Args:
            t: Input parameter (0 to 1)
            
        Returns:
            Smoothed parameter (0 to 1)
        """
        # Smooth S-curve using cosine interpolation
        return 0.5 * (1 - math.cos(math.pi * t))
    
    def calculate_workspace_volume(self, resolution: int = 10) -> Dict[str, Any]:
        """
        Calculate the robot's workspace volume and characteristics.
        
        Args:
            resolution: Grid resolution for workspace sampling
            
        Returns:
            Dictionary with workspace information
        """
        try:
            reachable_points = 0
            total_points = 0
            
            x_step = (self.x_range[1] - self.x_range[0]) / resolution
            y_step = (self.y_range[1] - self.y_range[0]) / resolution
            z_step = (self.z_range[1] - self.z_range[0]) / resolution
            
            for i in range(resolution + 1):
                for j in range(resolution + 1):
                    for k in range(resolution + 1):
                        x = self.x_range[0] + i * x_step
                        y = self.y_range[0] + j * y_step
                        z = self.z_range[0] + k * z_step
                        
                        total_points += 1
                        
                        if self.is_position_reachable(x, y, z):
                            # Try inverse kinematics to verify
                            joint_angles = self.inverse_kinematics(x, y, z)
                            if joint_angles is not None:
                                reachable_points += 1
            
            workspace_info = {
                "total_points_sampled": total_points,
                "reachable_points": reachable_points,
                "workspace_coverage": (reachable_points / total_points * 100) if total_points > 0 else 0,
                "max_theoretical_reach": self.total_reach,
                "configured_max_reach": self.max_reach,
                "workspace_bounds": {
                    "x_range": self.x_range,
                    "y_range": self.y_range,
                    "z_range": self.z_range
                },
                "resolution": resolution
            }
            
            self.logger.info(f"Workspace analysis: {workspace_info['workspace_coverage']:.1f}% coverage")
            return workspace_info
            
        except Exception as e:
            self.logger.error(f"Workspace calculation failed: {e}")
            return {}
    
    def get_joint_configuration_score(self, joint_angles: List[float]) -> float:
        """
        Score a joint configuration based on various criteria.
        
        Args:
            joint_angles: Joint angles in degrees
            
        Returns:
            Score from 0 (worst) to 1 (best)
        """
        try:
            score = 1.0
            
            # Penalize extreme joint angles
            for i, angle in enumerate(joint_angles):
                # Normalize angle to [-180, 180] range
                normalized_angle = ((angle + 180) % 360) - 180
                extreme_penalty = abs(normalized_angle) / 180.0
                score *= (1.0 - 0.3 * extreme_penalty)
            
            # Check for singularities (simplified check)
            if len(joint_angles) >= 2:
                # Penalize configurations near singular positions
                shoulder_angle = joint_angles[1] if len(joint_angles) > 1 else 0
                elbow_angle = joint_angles[2] if len(joint_angles) > 2 else 0
                
                # Avoid fully extended arm (singularity)
                if abs(elbow_angle) < 10:  # Nearly straight
                    score *= 0.5
            
            # Prefer configurations closer to "home" position
            home_angles = [0, -90, 0]  # Typical home position
            for i, (angle, home) in enumerate(zip(joint_angles[:len(home_angles)], home_angles)):
                deviation = abs(angle - home) / 180.0
                score *= (1.0 - 0.1 * deviation)
            
            return max(0.0, min(1.0, score))
            
        except Exception as e:
            self.logger.error(f"Configuration scoring failed: {e}")
            return 0.0
    
    def find_multiple_ik_solutions(self, x: float, y: float, z: float, 
                                  max_solutions: int = 4) -> List[List[float]]:
        """
        Find multiple inverse kinematics solutions for redundant configurations.
        
        Args:
            x, y, z: Target position
            max_solutions: Maximum number of solutions to find
            
        Returns:
            List of joint angle solutions
        """
        solutions = []
        
        try:
            # For a 3-DOF arm, there are typically 2 main solutions:
            # 1. Elbow up configuration
            # 2. Elbow down configuration
            
            # Primary solution (elbow up)
            primary_solution = self.inverse_kinematics(x, y, z)
            if primary_solution:
                solutions.append(primary_solution)
            
            # Alternative solution (elbow down) - modify elbow angle
            if primary_solution and len(primary_solution) >= 3:
                alt_solution = primary_solution.copy()
                alt_solution[2] = -alt_solution[2]  # Flip elbow angle
                
                # Verify this solution works
                verification_pos = self.forward_kinematics(alt_solution)
                if verification_pos:
                    error = math.sqrt(
                        (verification_pos.x - x)**2 + 
                        (verification_pos.y - y)**2 + 
                        (verification_pos.z - z)**2
                    )
                    if error < 5.0:  # Within 5mm tolerance
                        solutions.append(alt_solution)
            
            # Score and sort solutions
            scored_solutions = []
            for solution in solutions:
                score = self.get_joint_configuration_score(solution)
                scored_solutions.append((score, solution))
            
            # Sort by score (best first)
            scored_solutions.sort(reverse=True, key=lambda x: x[0])
            
            # Return top solutions
            final_solutions = [sol for score, sol in scored_solutions[:max_solutions]]
            
            self.logger.debug(f"Found {len(final_solutions)} IK solutions for position ({x:.1f}, {y:.1f}, {z:.1f})")
            return final_solutions
            
        except Exception as e:
            self.logger.error(f"Multiple IK solutions search failed: {e}")
            return solutions
    
    def get_configuration_info(self) -> Dict[str, Any]:
        """
        Get kinematics configuration information.
        
        Returns:
            Dictionary with configuration details
        """
        return {
            "degrees_of_freedom": self.dof,
            "link_lengths": self.link_lengths,
            "joint_offsets": self.joint_offsets,
            "end_effector_length": self.end_effector_length,
            "base_height": self.base_height,
            "total_reach": self.total_reach,
            "max_reach": self.max_reach,
            "min_reach": self.min_reach,
            "workspace": {
                "x_range": self.x_range,
                "y_range": self.y_range,
                "z_range": self.z_range
            },
            "constraints": {
                "max_joint_velocity": self.max_joint_velocity,
                "collision_avoidance": self.collision_avoidance
            }
        }
