#!/usr/bin/env python3
'''
Enhanced Servo Diagnostic and Fix Script
This script will identify and try to fix common servo movement issues
'''
import time
import serial
import fashionstar_uart_sdk as uservo

SERVO_PORT_NAME = '/dev/tty.usbserial-14110' 
SERVO_BAUDRATE = 1000000 
SERVO_ID = 0

def interpret_status(status_byte):
    """Interpret the servo status byte"""
    if status_byte is None:
        return "Status query failed"
    
    status_msg = []
    if status_byte & 0x01:
        status_msg.append("⚡ Executing command")
    if status_byte & 0x02:
        status_msg.append("❌ Command error")
    if status_byte & 0x04:
        status_msg.append("🚫 Stall error (blocked)")
    if status_byte & 0x08:
        status_msg.append("⚠️ Voltage too high")
    if status_byte & 0x10:
        status_msg.append("⚠️ Voltage too low")
    if status_byte & 0x20:
        status_msg.append("🌡️ Temperature too high")
        
    if not status_msg:
        status_msg.append("✅ Normal")
        
    return f"Status (0x{status_byte:02X}): " + ", ".join(status_msg)

def diagnose_and_fix_servo():
    """Comprehensive servo diagnosis and attempted fix"""
    print("=== Enhanced Servo Diagnosis ===")
    
    try:
        uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,
                           parity=serial.PARITY_NONE, stopbits=1, bytesize=8, timeout=1)
        control = uservo.UartServoManager(uart)
        
        print(f"✓ Connected to {SERVO_PORT_NAME}")
        
        # Step 1: Check servo status
        print(f"\\n1. Checking servo status...")
        status = control.query_status(SERVO_ID)
        print(f"   {interpret_status(status)}")
        
        # Step 2: Check voltage and power
        print(f"\\n2. Checking power supply...")
        voltage = control.query_voltage(SERVO_ID)
        current = control.query_current(SERVO_ID)
        power = control.query_power(SERVO_ID)
        temperature = control.query_temperature(SERVO_ID)
        
        print(f"   Voltage: {voltage}V" if voltage else "   Voltage: Query failed")
        print(f"   Current: {current}A" if current else "   Current: Query failed")
        print(f"   Power: {power}W" if power else "   Power: Query failed")
        print(f"   Temperature: {temperature}°C" if temperature else "   Temperature: Query failed")
        
        # Step 3: Try to clear any errors by disabling/enabling damping
        print(f"\\n3. Attempting to clear errors...")
        print("   Disabling damping (enable servo)...")
        control.set_damping(SERVO_ID, 0)  # 0 power = disable damping = enable servo
        time.sleep(0.5)
        
        # Step 4: Test movement with different parameters
        print(f"\\n4. Testing movement with various parameters...")
        
        initial_angle = control.query_servo_angle(SERVO_ID)
        print(f"   Initial angle: {initial_angle}°")
        
        # Test A: Slow movement with long interval
        print("   Test A: Slow movement (5 second interval)...")
        target_angle = initial_angle + 20 if initial_angle < 160 else initial_angle - 20
        control.set_servo_angle(SERVO_ID, target_angle, interval=5000)
        time.sleep(6)
        new_angle = control.query_servo_angle(SERVO_ID)
        print(f"   Result: {initial_angle}° → {new_angle}° (change: {abs(new_angle - initial_angle):.1f}°)")
        
        if abs(new_angle - initial_angle) > 1:
            print("   ✅ SUCCESS! Servo is now moving")
            return True
        
        # Test B: Movement with specified velocity
        print("   Test B: Movement with velocity control...")
        target_angle = initial_angle - 30 if initial_angle > -150 else initial_angle + 30
        control.set_servo_angle(SERVO_ID, target_angle, velocity=50.0, t_acc=500, t_dec=500)
        time.sleep(5)
        new_angle = control.query_servo_angle(SERVO_ID)
        print(f"   Result: {initial_angle}° → {new_angle}° (change: {abs(new_angle - initial_angle):.1f}°)")
        
        if abs(new_angle - initial_angle) > 1:
            print("   ✅ SUCCESS! Servo is now moving")
            return True
            
        # Test C: Movement with higher power
        print("   Test C: Movement with higher power...")
        target_angle = initial_angle + 15 if initial_angle < 165 else initial_angle - 15
        control.set_servo_angle(SERVO_ID, target_angle, power=800)  # Higher power
        time.sleep(3)
        new_angle = control.query_servo_angle(SERVO_ID)
        print(f"   Result: {initial_angle}° → {new_angle}° (change: {abs(new_angle - initial_angle):.1f}°)")
        
        if abs(new_angle - initial_angle) > 1:
            print("   ✅ SUCCESS! Servo is now moving")
            return True
        
        # If still not working, check final status
        print(f"\\n5. Final status check...")
        final_status = control.query_status(SERVO_ID)
        print(f"   {interpret_status(final_status)}")
        
        print("\\n❌ Servo still not responding to movement commands")
        return False
        
    except Exception as e:
        print(f"❌ Error during diagnosis: {e}")
        return False
    finally:
        if 'uart' in locals():
            uart.close()

def print_recommendations(servo_working):
    """Print specific recommendations based on test results"""
    print(f"\\n=== Recommendations ===")
    
    if not servo_working:
        print("🔧 Hardware troubleshooting steps:")
        print("1. 🔌 Check power supply:")
        print("   - Ensure 6-12V DC supply with sufficient current (>1A)")
        print("   - Verify power connections are secure")
        print("   - Check if power LED is on (if available)")
        print("\\n2. 🔗 Check connections:")
        print("   - Verify TX/RX are not swapped")
        print("   - Ensure good ground connection")
        print("   - Check for loose wires")
        print("\\n3. 🔧 Mechanical issues:")
        print("   - Check if servo shaft can move freely by hand")
        print("   - Remove any attached loads temporarily")
        print("   - Look for physical obstructions")
        print("\\n4. 📋 Servo configuration:")
        print("   - Try a different servo ID (1-7)")
        print("   - Check if servo needs initialization/calibration")
        print("   - Verify servo is not in a locked/disabled mode")
    else:
        print("✅ Servo is working! If movement seems slow or weak:")
        print("1. Increase power supply voltage (within servo specs)")
        print("2. Reduce mechanical load")
        print("3. Use higher power setting in commands")

if __name__ == "__main__":
    success = diagnose_and_fix_servo()
    print_recommendations(success)
