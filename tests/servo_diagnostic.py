#!/usr/bin/env python3
'''
Servo Diagnostic Script
Helps identify why the servo is not moving
'''
import time
import serial
import fashionstar_uart_sdk as uservo

SERVO_PORT_NAME = '/dev/tty.usbserial-14110' 
SERVO_BAUDRATE = 1000000 
SERVO_ID = 0

def test_servo_communication():
    """Test basic servo communication and status"""
    print("=== Servo Diagnostic Test ===")
    
    try:
        # Initialize serial connection
        uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,
                           parity=serial.PARITY_NONE, stopbits=1, bytesize=8, timeout=1)
        control = uservo.UartServoManager(uart)
        
        print(f"✓ Serial connection established on {SERVO_PORT_NAME}")
        
        # Test 1: Ping servo
        print(f"\n1. Testing servo ping (ID: {SERVO_ID})...")
        try:
            # Query current angle to test communication
            current_angle = control.query_servo_angle(SERVO_ID)
            print(f"✓ Servo responds! Current angle: {current_angle}°")
        except Exception as e:
            print(f"✗ Servo ping failed: {e}")
            return False
            
        # Test 2: Check servo status if available
        print(f"\n2. Testing servo status...")
        try:
            # Try to get servo status (if supported by SDK)
            status = control.query_servo_status(SERVO_ID) if hasattr(control, 'query_servo_status') else None
            if status:
                print(f"✓ Servo status: {status}")
            else:
                print("ℹ Status query not available in SDK")
        except Exception as e:
            print(f"ℹ Status query failed: {e}")
        
        # Test 3: Small angle change test
        print(f"\n3. Testing small angle changes...")
        initial_angle = control.query_servo_angle(SERVO_ID)
        print(f"Initial angle: {initial_angle}°")
        
        # Try a small movement
        target_angle = initial_angle + 10 if initial_angle < 170 else initial_angle - 10
        print(f"Setting angle to: {target_angle}°")
        control.set_servo_angle(SERVO_ID, target_angle, interval=2000)
        
        time.sleep(3)  # Wait longer for movement
        
        new_angle = control.query_servo_angle(SERVO_ID)
        print(f"New angle: {new_angle}°")
        
        angle_diff = abs(new_angle - initial_angle)
        if angle_diff > 1:
            print(f"✓ Servo moved! Angle change: {angle_diff}°")
        else:
            print(f"✗ Servo did not move significantly. Angle change: {angle_diff}°")
            
        # Test 4: Try different servo IDs
        print(f"\n4. Scanning for other servo IDs...")
        found_servos = []
        for test_id in range(8):  # Test IDs 0-7
            try:
                angle = control.query_servo_angle(test_id)
                found_servos.append((test_id, angle))
                print(f"  ID {test_id}: {angle}°")
            except:
                pass
                
        if found_servos:
            print(f"✓ Found {len(found_servos)} responding servo(s)")
        else:
            print("✗ No servos found responding")
            
        uart.close()
        return True
        
    except serial.SerialException as e:
        print(f"✗ Serial connection failed: {e}")
        print("  Check:")
        print("  - Is the device connected?")
        print("  - Is the port name correct?")
        print("  - Are permissions set correctly?")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False

def check_power_and_connections():
    """Provide checklist for hardware issues"""
    print(f"\n=== Hardware Checklist ===")
    print("Please verify:")
    print("□ Servo has adequate power supply (usually 6-12V)")
    print("□ Power supply can provide sufficient current (>1A for larger servos)")
    print("□ All connections are secure")
    print("□ Ground connections are properly connected")
    print("□ TX/RX lines are not swapped")
    print("□ Servo is not physically jammed or obstructed")
    print("□ Servo LED indicators (if any) show normal operation")

if __name__ == "__main__":
    success = test_servo_communication()
    if not success:
        check_power_and_connections()
    
    print(f"\n=== Recommendations ===")
    if not success:
        print("1. Check hardware connections and power supply")
        print("2. Verify servo ID matches your hardware configuration")
        print("3. Try a different baudrate (115200, 500000)")
    else:
        print("1. If servo responds but doesn't move, check power supply")
        print("2. Try enabling the servo or changing control mode")
        print("3. Check for mechanical obstructions")
