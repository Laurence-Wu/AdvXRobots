#!/usr/bin/env python3
"""
Integrated Servo Test Suite for EmoRobots Visual Control System
===============================================================
Comprehensive testing for servo detection, movement, and visual control integration

Features:
- Servo ID discovery and availability testing
- Movement capability testing for servos 1 & 2 (face tracking)
- Configuration validation
- Visual robot control system validation
- Mock testing when hardware unavailable

Author: EmoRobots Project
Date: July 25, 2025
"""

import os
import sys
import time
import json
import serial
from datetime import datetime
from typing import Dict, List, Optional, Tuple

# Add SDK paths
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'servo_sdk', 'python'))

# Try to import servo SDK
try:
    import fashionstar_uart_sdk as servo_sdk
    SDK_AVAILABLE = True
    print("✅ Servo SDK loaded")
except ImportError:
    SDK_AVAILABLE = False
    servo_sdk = None
    print("⚠️  Servo SDK not available - using mock implementation")

# Import configuration if available
try:
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
    from config import SERIAL_CONFIG, SERVO_CONFIG
    CONFIG_AVAILABLE = True
except ImportError:
    CONFIG_AVAILABLE = False
    # Default configuration fallback
    SERIAL_CONFIG = {'port': None, 'baudrate': 115200, 'timeout': 0.1}
    SERVO_CONFIG = {
        'scan_range': [0, 1, 2, 3, 4, 5],
        'initial_positions': {1: -90, 2: -90, 3: 0},
        'power_settings': {'active_servos': [1, 2, 3], 'zero_power_servos': [0, 4]},
        'motion_params': {'angle_tolerance': 2.0}
    }


class MockServoManager:
    """Mock servo manager for testing without hardware"""
    
    def __init__(self):
        print("🎮 Initializing Mock Servo Manager")
        # Mock servos based on expected configuration
        self.mock_servos = {0: True, 1: True, 2: True, 3: True, 4: True, 5: False}
        self.mock_angles = {0: 0.0, 1: -90.0, 2: -90.0, 3: 0.0, 4: 0.0}
        self.mock_power = {0: 0, 1: 8000, 2: 8000, 3: 8000, 4: 0}
    
    def ping(self, servo_id: int) -> bool:
        return self.mock_servos.get(servo_id, False)
    
    def query_servo_angle(self, servo_id: int) -> float:
        return self.mock_angles.get(servo_id, 0.0)
    
    def set_servo_angle(self, servo_id: int, angle: float, interval=500, power=8000, **kwargs):
        if servo_id in self.mock_angles:
            self.mock_angles[servo_id] = angle
            print(f"🎮 Mock: Servo {servo_id} → {angle}° (time: {interval}ms, power: {power}mW)")
            time.sleep(0.1)  # Simulate movement time
            return True
        return False
    
    def set_damping(self, servo_id: int, power: int = 0):
        if servo_id in self.mock_servos:
            self.mock_power[servo_id] = power
            print(f"🎮 Mock: Servo {servo_id} damping set to {power}")
            return True
        return False
    
    def query_voltage(self, servo_id: int) -> float:
        return 8.4
    
    def query_current(self, servo_id: int) -> float:
        return 0.5
    
    def query_temperature(self, servo_id: int) -> float:
        return 35.0


class IntegratedServoTester:
    """Integrated servo testing class"""
    
    def __init__(self, use_hardware: bool = True, port: str = None, baudrate: int = None):
        self.use_hardware = use_hardware and SDK_AVAILABLE
        self.port = port or SERIAL_CONFIG.get('port')
        self.baudrate = baudrate or SERIAL_CONFIG.get('baudrate', 1000000)
        
        # Initialize connection and manager
        self.uart = None
        self.servo_manager = None
        self.connected = False
        
        # Test results
        self.test_results = {
            'timestamp': datetime.now().isoformat(),
            'hardware_mode': self.use_hardware,
            'detected_servos': {},
            'movement_tests': {},
            'configuration_status': {},
            'visual_control_ready': False,
            'errors': []
        }
        
        print(f"🔧 Test Mode: {'Hardware' if self.use_hardware else 'Mock'}")
    
    def connect(self) -> bool:
        """Establish connection to servo system"""
        try:
            if self.use_hardware:
                if not self.port or not self.baudrate:
                    print("❌ Hardware mode requires port and baudrate configuration")
                    return False
                
                print(f"🔌 Connecting to {self.port} @ {self.baudrate} baud...")
                self.uart = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=SERIAL_CONFIG.get('timeout', 0.1),
                    parity=serial.PARITY_NONE,
                    stopbits=1,
                    bytesize=8
                )
                
                self.servo_manager = servo_sdk.UartServoManager(self.uart)
                print("✅ Hardware connection established")
            else:
                self.servo_manager = MockServoManager()
                print("✅ Mock connection established")
            
            self.connected = True
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            self.test_results['errors'].append(f"Connection error: {e}")
            return False
    
    def disconnect(self):
        """Close connection"""
        try:
            if self.uart and hasattr(self.uart, 'close'):
                self.uart.close()
            self.connected = False
            print("🔌 Connection closed")
        except Exception as e:
            print(f"⚠️  Disconnect error: {e}")
    
    def test_servo_detection(self) -> Dict[int, Dict]:
        """Test servo detection and basic status"""
        print("\n🔍 Servo Detection Test")
        print("-" * 40)
        
        detected_servos = {}
        test_ids = SERVO_CONFIG['scan_range']
        
        for servo_id in test_ids:
            try:
                if self.servo_manager.ping(servo_id):
                    angle = self.servo_manager.query_servo_angle(servo_id)
                    detected_servos[servo_id] = {
                        'id': servo_id,
                        'online': True,
                        'current_angle': angle,
                        'status': 'detected'
                    }
                    print(f"✅ Servo {servo_id}: Online (angle: {angle:.1f}°)")
                else:
                    print(f"❌ Servo {servo_id}: Not found")
            except Exception as e:
                print(f"⚠️  Servo {servo_id}: Error - {e}")
                self.test_results['errors'].append(f"Detection error servo {servo_id}: {e}")
        
        self.test_results['detected_servos'] = detected_servos
        print(f"\n📊 Detection Summary: {len(detected_servos)}/{len(test_ids)} servos found")
        return detected_servos
    
    def test_servo_initialization(self, detected_servos: Dict) -> bool:
        """Test servo initialization according to configuration"""
        print("\n🚀 Servo Initialization Test")
        print("-" * 40)
        
        initial_positions = SERVO_CONFIG['initial_positions']
        zero_power_servos = SERVO_CONFIG['power_settings']['zero_power_servos']
        
        success_count = 0
        total_tests = len(initial_positions) + len(zero_power_servos)
        
        # Test initial position setting
        for servo_id, target_angle in initial_positions.items():
            if servo_id in detected_servos:
                try:
                    success = self.servo_manager.set_servo_angle(
                        servo_id, target_angle, interval=500, power=8000
                    )
                    if success:
                        # Verify position
                        time.sleep(0.2)
                        actual_angle = self.servo_manager.query_servo_angle(servo_id)
                        error = abs(actual_angle - target_angle) if actual_angle is not None else float('inf')
                        
                        if error < 5.0:  # 5 degree tolerance
                            detected_servos[servo_id]['initialized_angle'] = actual_angle
                            detected_servos[servo_id]['target_angle'] = target_angle
                            detected_servos[servo_id]['angle_error'] = error
                            print(f"✅ Servo {servo_id}: {target_angle}° → {actual_angle:.1f}° (error: {error:.1f}°)")
                            success_count += 1
                        else:
                            print(f"❌ Servo {servo_id}: Position error too large ({error:.1f}°)")
                    else:
                        print(f"❌ Servo {servo_id}: Failed to set position")
                except Exception as e:
                    print(f"⚠️  Servo {servo_id}: Initialization error - {e}")
                    self.test_results['errors'].append(f"Init error servo {servo_id}: {e}")
            else:
                print(f"⚠️  Servo {servo_id}: Not detected, skipping initialization")
        
        # Test zero power setting
        for servo_id in zero_power_servos:
            if servo_id in detected_servos:
                try:
                    success = self.servo_manager.set_damping(servo_id, 0)
                    if success:
                        detected_servos[servo_id]['power_mode'] = 'zero_power'
                        print(f"🔋 Servo {servo_id}: Zero power set")
                        success_count += 1
                    else:
                        print(f"❌ Servo {servo_id}: Failed to set zero power")
                except Exception as e:
                    print(f"⚠️  Servo {servo_id}: Power setting error - {e}")
        
        initialization_success = success_count >= (total_tests * 0.8)  # 80% success rate
        print(f"\n📊 Initialization: {success_count}/{total_tests} successful")
        return initialization_success
    
    def test_face_tracking_movement(self, detected_servos: Dict) -> Dict:
        """Test movement capabilities for face tracking (servos 1 & 2)"""
        print("\n🎯 Face Tracking Movement Test")
        print("-" * 40)
        
        movement_results = {}
        face_tracking_servos = [1, 2]
        
        # Test movement patterns that simulate face tracking
        test_movements = [
            (-75, "Look up-left"),
            (-90, "Center position"),
            (-105, "Look down-right"),
            (-90, "Return to center")
        ]
        
        for servo_id in face_tracking_servos:
            if servo_id not in detected_servos:
                print(f"⚠️  Servo {servo_id}: Not available for face tracking")
                continue
            
            print(f"\n🤖 Testing Servo {servo_id} face tracking movements:")
            movement_results[servo_id] = []
            
            for target_angle, description in test_movements:
                try:
                    print(f"  {description}: {target_angle}°")
                    
                    # Apply angle limits
                    angle_limits = SERVO_CONFIG['angle_limits'].get(f'servo_{servo_id}_range', (-180, 180))
                    clamped_angle = max(angle_limits[0], min(angle_limits[1], target_angle))
                    
                    if clamped_angle != target_angle:
                        print(f"    Clamped to limits: {clamped_angle}°")
                    
                    # Execute movement
                    success = self.servo_manager.set_servo_angle(servo_id, clamped_angle, interval=300)
                    
                    if success:
                        time.sleep(0.4)  # Wait for movement
                        actual_angle = self.servo_manager.query_servo_angle(servo_id)
                        error = abs(actual_angle - clamped_angle) if actual_angle is not None else float('inf')
                        
                        result = {
                            'target': clamped_angle,
                            'actual': actual_angle,
                            'error': error,
                            'success': error < 5.0,
                            'description': description
                        }
                        movement_results[servo_id].append(result)
                        
                        status = "✅" if result['success'] else "❌"
                        print(f"    {status} Result: {actual_angle:.1f}° (error: {error:.1f}°)")
                    else:
                        print(f"    ❌ Movement failed")
                        movement_results[servo_id].append({
                            'target': clamped_angle,
                            'actual': None,
                            'error': float('inf'),
                            'success': False,
                            'description': description
                        })
                
                except Exception as e:
                    print(f"    ⚠️  Movement error: {e}")
                    self.test_results['errors'].append(f"Movement error servo {servo_id}: {e}")
        
        self.test_results['movement_tests'] = movement_results
        
        # Calculate success rate
        total_movements = sum(len(results) for results in movement_results.values())
        successful_movements = sum(
            sum(1 for result in results if result['success']) 
            for results in movement_results.values()
        )
        
        success_rate = (successful_movements / total_movements * 100) if total_movements > 0 else 0
        print(f"\n📊 Movement Test Summary: {successful_movements}/{total_movements} successful ({success_rate:.1f}%)")
        
        return movement_results
    
    def test_configuration_validation(self) -> Dict:
        """Validate system configuration"""
        print("\n⚙️  Configuration Validation")
        print("-" * 40)
        
        config_status = {
            'config_file_available': CONFIG_AVAILABLE,
            'serial_configured': bool(SERIAL_CONFIG.get('port') and SERIAL_CONFIG.get('baudrate')),
            'servo_config_valid': True,
            'angle_limits_defined': True,
            'issues': []
        }
        
        # Check configuration completeness
        if not CONFIG_AVAILABLE:
            config_status['issues'].append("Configuration file not found")
            print("⚠️  Configuration file not available")
        else:
            print("✅ Configuration file loaded")
        
        # Check serial configuration
        if config_status['serial_configured']:
            print(f"✅ Serial: {SERIAL_CONFIG['port']} @ {SERIAL_CONFIG['baudrate']}")
        else:
            print("⚠️  Serial port/baudrate not configured")
            config_status['issues'].append("Serial port not configured")
        
        # Check servo configuration
        required_servos = SERVO_CONFIG['scan_range']
        active_servos = SERVO_CONFIG['power_settings']['active_servos']
        
        print(f"✅ Servo IDs configured: {required_servos}")
        print(f"✅ Active servos: {active_servos}")
        print(f"✅ Initial positions: {SERVO_CONFIG['initial_positions']}")
        
        # Check angle limits
        angle_limits = SERVO_CONFIG.get('angle_limits', {})
        for servo_id in [1, 2, 3]:  # Check key servos
            limit_key = f'servo_{servo_id}_range'
            if limit_key in angle_limits:
                limits = angle_limits[limit_key]
                print(f"✅ Servo {servo_id} limits: {limits[0]}° to {limits[1]}°")
            else:
                config_status['issues'].append(f"Angle limits not defined for servo {servo_id}")
                config_status['angle_limits_defined'] = False
        
        self.test_results['configuration_status'] = config_status
        
        # Overall configuration health
        config_health = len(config_status['issues']) == 0
        print(f"\n📊 Configuration Health: {'✅ Good' if config_health else '⚠️  Issues found'}")
        
        if config_status['issues']:
            print("Issues found:")
            for issue in config_status['issues']:
                print(f"  - {issue}")
        
        return config_status
    
    def test_visual_control_readiness(self, detected_servos: Dict, movement_results: Dict) -> bool:
        """Test if system is ready for visual robot control"""
        print("\n🎥 Visual Control Readiness Assessment")
        print("-" * 40)
        
        readiness_checks = {
            'servo_1_available': 1 in detected_servos,
            'servo_2_available': 2 in detected_servos,
            'servo_3_available': 3 in detected_servos,
            'movement_capability': False,
            'initialization_success': False,
            'configuration_ready': CONFIG_AVAILABLE
        }
        
        # Check movement capability
        if movement_results:
            movement_success_rate = 0
            total_tests = 0
            for servo_id, results in movement_results.items():
                if servo_id in [1, 2]:  # Face tracking servos
                    successful = sum(1 for r in results if r['success'])
                    total_tests += len(results)
                    movement_success_rate += successful
            
            movement_success_rate = (movement_success_rate / total_tests) if total_tests > 0 else 0
            readiness_checks['movement_capability'] = movement_success_rate >= 0.75  # 75% success rate
        
        # Check initialization success
        initialized_servos = sum(1 for servo in detected_servos.values() 
                               if 'initialized_angle' in servo)
        expected_init_servos = len(SERVO_CONFIG['initial_positions'])
        readiness_checks['initialization_success'] = initialized_servos >= expected_init_servos
        
        # Print readiness status
        for check, status in readiness_checks.items():
            status_icon = "✅" if status else "❌"
            check_name = check.replace('_', ' ').title()
            print(f"{status_icon} {check_name}: {'Ready' if status else 'Not Ready'}")
        
        # Overall readiness
        overall_ready = all(readiness_checks.values())
        self.test_results['visual_control_ready'] = overall_ready
        
        print(f"\n🎯 Visual Control System: {'✅ READY' if overall_ready else '❌ NOT READY'}")
        
        if overall_ready:
            print("✨ System is ready for face tracking and visual robot control!")
        else:
            print("⚠️  Please address the issues above before using visual control")
        
        return overall_ready
    
    def save_test_report(self) -> str:
        """Save comprehensive test report"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_file = f"integrated_servo_test_report_{timestamp}.json"
        
        try:
            with open(report_file, 'w') as f:
                json.dump(self.test_results, f, indent=2, default=str)
            print(f"\n💾 Test report saved: {report_file}")
            return report_file
        except Exception as e:
            print(f"❌ Failed to save report: {e}")
            return ""
    
    def run_complete_test_suite(self) -> Dict:
        """Run the complete integrated test suite"""
        print("🚀 Starting Integrated Servo Test Suite")
        print("=" * 60)
        
        if not self.connect():
            print("❌ Failed to establish connection. Test aborted.")
            return self.test_results
        
        try:
            # Test 1: Servo Detection
            detected_servos = self.test_servo_detection()
            
            if not detected_servos:
                print("❌ No servos detected. Cannot continue with movement tests.")
                return self.test_results
            
            # Test 2: Servo Initialization  
            init_success = self.test_servo_initialization(detected_servos)
            
            # Test 3: Face Tracking Movement Tests
            movement_results = self.test_face_tracking_movement(detected_servos)
            
            # Test 4: Configuration Validation
            config_status = self.test_configuration_validation()
            
            # Test 5: Visual Control Readiness
            ready_for_visual = self.test_visual_control_readiness(detected_servos, movement_results)
            
            # Final Summary
            print("\n" + "=" * 60)
            print("              INTEGRATED TEST SUMMARY")
            print("=" * 60)
            print(f"Test Mode: {'Hardware' if self.use_hardware else 'Mock Simulation'}")
            print(f"Servos Detected: {len(detected_servos)}")
            print(f"Configuration: {'✅ Valid' if CONFIG_AVAILABLE else '⚠️  Default'}")
            print(f"Movement Tests: {'✅ Passed' if movement_results else '❌ Failed'}")
            print(f"Visual Control Ready: {'✅ Yes' if ready_for_visual else '❌ No'}")
            print(f"Total Errors: {len(self.test_results['errors'])}")
            
            if self.test_results['errors']:
                print(f"\nErrors encountered:")
                for i, error in enumerate(self.test_results['errors'][:5], 1):
                    print(f"  {i}. {error}")
                if len(self.test_results['errors']) > 5:
                    print(f"  ... and {len(self.test_results['errors']) - 5} more")
            
            print("=" * 60)
            
            # Save report
            self.save_test_report()
            
            return self.test_results
            
        except Exception as e:
            print(f"❌ Test suite error: {e}")
            self.test_results['errors'].append(f"Test suite error: {e}")
            return self.test_results
        
        finally:
            self.disconnect()


def main():
    """Main function for integrated servo testing"""
    print("🤖 EmoRobots Integrated Servo Test Suite")
    print("=" * 50)
    print("Choose test mode:")
    print("1. Hardware Testing (requires connected servos)")
    print("2. Mock Testing (simulation mode)")
    print("3. Exit")
    print("=" * 50)
    
    try:
        choice = input("Enter choice (1-3): ").strip()
    except KeyboardInterrupt:
        print("\nTest cancelled.")
        return
    
    if choice == "1":
        # Hardware testing mode
        print("\n🔧 Hardware Testing Mode")
        port = input("Enter serial port (or press Enter for default): ").strip()
        if not port:
            port = SERIAL_CONFIG.get('port')
        
        if not port:
            print("❌ No serial port configured. Using mock mode instead.")
            tester = IntegratedServoTester(use_hardware=False)
        else:
            tester = IntegratedServoTester(use_hardware=True, port=port)
    
    elif choice == "2":
        # Mock testing mode
        print("\n🎮 Mock Testing Mode")
        tester = IntegratedServoTester(use_hardware=False)
    
    elif choice == "3":
        print("Exiting...")
        return
    
    else:
        print("Invalid choice. Using mock mode.")
        tester = IntegratedServoTester(use_hardware=False)
    
    # Run the complete test suite
    results = tester.run_complete_test_suite()
    
    # Final recommendations
    print("\n🎯 Next Steps:")
    if results.get('visual_control_ready'):
        print("✅ Your system is ready for visual robot control!")
        print("   Run: python3 visual_robot_control.py")
    else:
        print("⚠️  Please address the configuration issues before proceeding")
        print("   Check servo connections and configuration files")
    
    print("\nTest completed. 🎉")


if __name__ == "__main__":
    main()
