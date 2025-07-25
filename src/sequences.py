#!/usr/bin/env python3
"""
Sequences Module for EmoRobots Robot Arm
=======================================

This module manages pre-programmed movement sequences and behaviors
for the robot arm system.

Author: EmoRobots Team
Date: July 25, 2025
"""

import time
import logging
import threading
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass


@dataclass
class SequenceStep:
    """Data class for a sequence step."""
    servo_id: int
    angle: float
    duration: int  # milliseconds
    power: Optional[int] = None
    wait_for_completion: bool = True
    description: str = ""


@dataclass
class SequenceInfo:
    """Data class for sequence information."""
    name: str
    description: str
    steps: List[SequenceStep]
    total_duration: float
    loops: int = 1
    interruptible: bool = True


class SequenceManager:
    """
    Sequence management system for pre-programmed robot arm movements.
    
    Handles sequence definition, execution, and coordination with
    the hardware interface.
    """
    
    def __init__(self, hardware_interface, sequences_config: Dict[str, Any]):
        """
        Initialize sequence manager.
        
        Args:
            hardware_interface: Hardware interface instance
            sequences_config: Sequences configuration dictionary
        """
        self.hardware = hardware_interface
        self.config = sequences_config
        self.logger = logging.getLogger(__name__)
        
        # Sequence storage
        self.sequences = {}
        self.current_sequence = None
        self.sequence_thread = None
        
        # Execution state
        self.executing = False
        self.paused = False
        self.stop_requested = False
        self.current_step = 0
        self.total_steps = 0
        
        # Callbacks
        self.on_sequence_start: Optional[Callable] = None
        self.on_sequence_complete: Optional[Callable] = None
        self.on_sequence_error: Optional[Callable] = None
        self.on_step_complete: Optional[Callable] = None
        
        # Load sequences from configuration
        self._load_sequences_from_config()
        
        self.logger.info(f"Sequence manager initialized with {len(self.sequences)} sequences")
    
    def _load_sequences_from_config(self):
        """Load sequences from configuration dictionary."""
        try:
            for sequence_name, sequence_data in self.config.items():
                sequence_info = self._parse_sequence_config(sequence_name, sequence_data)
                if sequence_info:
                    self.sequences[sequence_name] = sequence_info
                    self.logger.debug(f"Loaded sequence: {sequence_name}")
                else:
                    self.logger.warning(f"Failed to parse sequence: {sequence_name}")
        
        except Exception as e:
            self.logger.error(f"Error loading sequences from config: {e}")
    
    def _parse_sequence_config(self, name: str, data: Dict[str, Any]) -> Optional[SequenceInfo]:
        """Parse sequence configuration data."""
        try:
            description = data.get("description", "")
            steps_data = data.get("steps", [])
            
            if not steps_data:
                self.logger.warning(f"Sequence {name} has no steps")
                return None
            
            steps = []
            total_duration = 0.0
            
            for step_data in steps_data:
                step = SequenceStep(
                    servo_id=step_data.get("servo_id"),
                    angle=step_data.get("angle"),
                    duration=step_data.get("duration", 1000),
                    power=step_data.get("power"),
                    wait_for_completion=step_data.get("wait", True),
                    description=step_data.get("description", "")
                )
                
                steps.append(step)
                total_duration += step.duration / 1000.0  # Convert to seconds
            
            sequence_info = SequenceInfo(
                name=name,
                description=description,
                steps=steps,
                total_duration=total_duration,
                loops=data.get("loops", 1),
                interruptible=data.get("interruptible", True)
            )
            
            return sequence_info
            
        except Exception as e:
            self.logger.error(f"Error parsing sequence {name}: {e}")
            return None
    
    def add_sequence(self, sequence_info: SequenceInfo):
        """
        Add a new sequence.
        
        Args:
            sequence_info: SequenceInfo object
        """
        self.sequences[sequence_info.name] = sequence_info
        self.logger.info(f"Added sequence: {sequence_info.name}")
    
    def remove_sequence(self, sequence_name: str) -> bool:
        """
        Remove a sequence.
        
        Args:
            sequence_name: Name of sequence to remove
            
        Returns:
            bool: True if sequence was removed
        """
        if sequence_name in self.sequences:
            del self.sequences[sequence_name]
            self.logger.info(f"Removed sequence: {sequence_name}")
            return True
        return False
    
    def get_available_sequences(self) -> List[str]:
        """
        Get list of available sequence names.
        
        Returns:
            List of sequence names
        """
        return list(self.sequences.keys())
    
    def get_sequence_info(self, sequence_name: str) -> Optional[SequenceInfo]:
        """
        Get information about a sequence.
        
        Args:
            sequence_name: Name of sequence
            
        Returns:
            SequenceInfo object or None if not found
        """
        return self.sequences.get(sequence_name)
    
    def execute_sequence(self, sequence_name: str, loops: Optional[int] = None) -> bool:
        """
        Execute a sequence by name.
        
        Args:
            sequence_name: Name of sequence to execute
            loops: Number of loops (overrides sequence default)
            
        Returns:
            bool: True if sequence started successfully
        """
        if self.executing:
            self.logger.warning("Sequence already executing")
            return False
        
        sequence_info = self.sequences.get(sequence_name)
        if not sequence_info:
            self.logger.error(f"Sequence not found: {sequence_name}")
            return False
        
        if not self.hardware:
            self.logger.error("Hardware interface not available")
            return False
        
        # Override loops if specified
        if loops is not None:
            sequence_info = SequenceInfo(
                name=sequence_info.name,
                description=sequence_info.description,
                steps=sequence_info.steps,
                total_duration=sequence_info.total_duration,
                loops=loops,
                interruptible=sequence_info.interruptible
            )
        
        self.logger.info(f"Starting sequence: {sequence_name} ({len(sequence_info.steps)} steps, {loops or sequence_info.loops} loops)")
        
        # Reset execution state
        self.current_sequence = sequence_info
        self.executing = True
        self.paused = False
        self.stop_requested = False
        self.current_step = 0
        self.total_steps = len(sequence_info.steps) * sequence_info.loops
        
        # Start execution thread
        self.sequence_thread = threading.Thread(
            target=self._execute_sequence_thread,
            args=(sequence_info,),
            daemon=True
        )
        self.sequence_thread.start()
        
        # Call start callback
        if self.on_sequence_start:
            try:
                self.on_sequence_start(sequence_name)
            except Exception as e:
                self.logger.error(f"Error in sequence start callback: {e}")
        
        return True
    
    def _execute_sequence_thread(self, sequence_info: SequenceInfo):
        """Execute sequence in separate thread."""
        try:
            for loop in range(sequence_info.loops):
                if self.stop_requested:
                    break
                
                self.logger.debug(f"Starting loop {loop + 1}/{sequence_info.loops}")
                
                for step_index, step in enumerate(sequence_info.steps):
                    if self.stop_requested:
                        break
                    
                    # Handle pause
                    while self.paused and not self.stop_requested:
                        time.sleep(0.1)
                    
                    if self.stop_requested:
                        break
                    
                    # Execute step
                    self.current_step = loop * len(sequence_info.steps) + step_index + 1
                    
                    success = self._execute_step(step)
                    
                    if not success:
                        self.logger.error(f"Step {step_index + 1} failed in sequence {sequence_info.name}")
                        if self.on_sequence_error:
                            try:
                                self.on_sequence_error(sequence_info.name, step_index, "Step execution failed")
                            except Exception as e:
                                self.logger.error(f"Error in sequence error callback: {e}")
                        break
                    
                    # Call step completion callback
                    if self.on_step_complete:
                        try:
                            self.on_step_complete(sequence_info.name, step_index, self.current_step, self.total_steps)
                        except Exception as e:
                            self.logger.error(f"Error in step complete callback: {e}")
                
                # Small delay between loops
                if loop < sequence_info.loops - 1 and not self.stop_requested:
                    time.sleep(0.5)
            
            # Sequence completed
            if not self.stop_requested:
                self.logger.info(f"Sequence completed: {sequence_info.name}")
                
                if self.on_sequence_complete:
                    try:
                        self.on_sequence_complete(sequence_info.name)
                    except Exception as e:
                        self.logger.error(f"Error in sequence complete callback: {e}")
            else:
                self.logger.info(f"Sequence stopped: {sequence_info.name}")
        
        except Exception as e:
            self.logger.error(f"Error during sequence execution: {e}")
            if self.on_sequence_error:
                try:
                    self.on_sequence_error(sequence_info.name, self.current_step, str(e))
                except Exception as callback_error:
                    self.logger.error(f"Error in sequence error callback: {callback_error}")
        
        finally:
            # Reset execution state
            self.executing = False
            self.paused = False
            self.stop_requested = False
            self.current_sequence = None
            self.current_step = 0
            self.total_steps = 0
    
    def _execute_step(self, step: SequenceStep) -> bool:
        """
        Execute a single sequence step.
        
        Args:
            step: SequenceStep to execute
            
        Returns:
            bool: True if step executed successfully
        """
        try:
            self.logger.debug(f"Executing step: Servo {step.servo_id} → {step.angle}° ({step.duration}ms)")
            
            # Execute servo movement
            success = self.hardware.move_servo(
                servo_id=step.servo_id,
                angle=step.angle,
                duration=step.duration,
                power=step.power
            )
            
            if not success:
                self.logger.error(f"Failed to move servo {step.servo_id}")
                return False
            
            # Wait for completion if required
            if step.wait_for_completion:
                # Wait for movement duration plus small buffer
                wait_time = step.duration / 1000.0 + 0.1
                time.sleep(wait_time)
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error executing step: {e}")
            return False
    
    def pause_sequence(self):
        """Pause the currently executing sequence."""
        if self.executing and not self.paused:
            self.paused = True
            self.logger.info("Sequence paused")
    
    def resume_sequence(self):
        """Resume a paused sequence."""
        if self.executing and self.paused:
            self.paused = False
            self.logger.info("Sequence resumed")
    
    def stop_sequence(self):
        """Stop the currently executing sequence."""
        if self.executing:
            self.stop_requested = True
            self.paused = False
            self.logger.info("Sequence stop requested")
            
            # Wait for thread to finish
            if self.sequence_thread and self.sequence_thread.is_alive():
                self.sequence_thread.join(timeout=2.0)
    
    def is_executing(self) -> bool:
        """Check if a sequence is currently executing."""
        return self.executing
    
    def is_paused(self) -> bool:
        """Check if sequence execution is paused."""
        return self.paused
    
    def get_execution_status(self) -> Dict[str, Any]:
        """
        Get current execution status.
        
        Returns:
            Dictionary with execution status information
        """
        if not self.executing:
            return {
                "executing": False,
                "sequence_name": None,
                "current_step": 0,
                "total_steps": 0,
                "progress": 0.0,
                "paused": False
            }
        
        progress = (self.current_step / self.total_steps * 100) if self.total_steps > 0 else 0
        
        return {
            "executing": True,
            "sequence_name": self.current_sequence.name if self.current_sequence else None,
            "current_step": self.current_step,
            "total_steps": self.total_steps,
            "progress": progress,
            "paused": self.paused,
            "estimated_remaining": self._estimate_remaining_time()
        }
    
    def _estimate_remaining_time(self) -> float:
        """Estimate remaining execution time in seconds."""
        if not self.executing or not self.current_sequence:
            return 0.0
        
        try:
            completed_steps = self.current_step
            remaining_steps = self.total_steps - completed_steps
            
            # Calculate average step duration
            total_step_duration = sum(step.duration for step in self.current_sequence.steps) / 1000.0
            avg_step_duration = total_step_duration / len(self.current_sequence.steps)
            
            return remaining_steps * avg_step_duration
            
        except Exception:
            return 0.0
    
    def create_sequence_from_positions(self, name: str, positions: List[Tuple[int, float, int]], 
                                     description: str = "") -> SequenceInfo:
        """
        Create a sequence from a list of servo positions.
        
        Args:
            name: Sequence name
            positions: List of (servo_id, angle, duration) tuples
            description: Sequence description
            
        Returns:
            SequenceInfo object
        """
        steps = []
        
        for servo_id, angle, duration in positions:
            step = SequenceStep(
                servo_id=servo_id,
                angle=angle,
                duration=duration,
                description=f"Move servo {servo_id} to {angle}°"
            )
            steps.append(step)
        
        total_duration = sum(step.duration for step in steps) / 1000.0
        
        sequence_info = SequenceInfo(
            name=name,
            description=description,
            steps=steps,
            total_duration=total_duration
        )
        
        self.add_sequence(sequence_info)
        return sequence_info
    
    def validate_sequence(self, sequence_name: str) -> Dict[str, Any]:
        """
        Validate a sequence for potential issues.
        
        Args:
            sequence_name: Name of sequence to validate
            
        Returns:
            Dictionary with validation results
        """
        sequence_info = self.sequences.get(sequence_name)
        if not sequence_info:
            return {"valid": False, "errors": ["Sequence not found"]}
        
        errors = []
        warnings = []
        
        # Check for empty sequence
        if not sequence_info.steps:
            errors.append("Sequence has no steps")
        
        # Validate each step
        for i, step in enumerate(sequence_info.steps):
            # Check servo ID validity
            if not isinstance(step.servo_id, int) or step.servo_id < 0:
                errors.append(f"Step {i+1}: Invalid servo ID {step.servo_id}")
            
            # Check angle validity
            if not isinstance(step.angle, (int, float)):
                errors.append(f"Step {i+1}: Invalid angle {step.angle}")
            elif abs(step.angle) > 180:
                warnings.append(f"Step {i+1}: Large angle {step.angle}° may exceed servo limits")
            
            # Check duration validity
            if not isinstance(step.duration, int) or step.duration <= 0:
                errors.append(f"Step {i+1}: Invalid duration {step.duration}")
            elif step.duration < 100:
                warnings.append(f"Step {i+1}: Very short duration {step.duration}ms")
            elif step.duration > 10000:
                warnings.append(f"Step {i+1}: Very long duration {step.duration}ms")
        
        # Check sequence timing
        if sequence_info.total_duration > 120:  # 2 minutes
            warnings.append(f"Long sequence duration: {sequence_info.total_duration:.1f}s")
        
        return {
            "valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings,
            "step_count": len(sequence_info.steps),
            "total_duration": sequence_info.total_duration
        }
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get sequence manager statistics.
        
        Returns:
            Dictionary with statistics
        """
        total_sequences = len(self.sequences)
        total_steps = sum(len(seq.steps) for seq in self.sequences.values())
        total_duration = sum(seq.total_duration for seq in self.sequences.values())
        
        return {
            "total_sequences": total_sequences,
            "total_steps": total_steps,
            "total_duration": total_duration,
            "average_steps_per_sequence": total_steps / total_sequences if total_sequences > 0 else 0,
            "average_duration_per_sequence": total_duration / total_sequences if total_sequences > 0 else 0,
            "currently_executing": self.executing,
            "sequences": list(self.sequences.keys())
        }
