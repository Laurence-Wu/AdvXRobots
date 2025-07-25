#!/usr/bin/env python3
'''
Servo Movement Guide - How to Make Servos Move
This script demonstrates the key techniques for making servos move reliably
'''
import time
import serial
import fashionstar_uart_sdk as uservo

# Configuration
SERVO_PORT_NAME = '/dev/tty.usbserial-14110'  # Adjust to your device
SERVO_BAUDRATE = 1000000 
SERVO_ID = 0

def demonstrate_servo_movement():
    """Demonstrate different ways to make a servo move"""
    
    try:
        # 1. Establish connection
        uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,
                           parity=serial.PARITY_NONE, stopbits=1, bytesize=8, timeout=1)
        control = uservo.UartServoManager(uart)
        print(f"✓ Connected to servo on {SERVO_PORT_NAME}")
        
        # 2. CRITICAL: Enable the servo (disable damping)
        print("\n🔧 Step 1: Enabling servo (disabling damping)...")
        control.set_damping(SERVO_ID, 0)  # 0 = disable damping = enable servo
        time.sleep(0.5)
        print("   ✅ Servo enabled!")
        
        current_angle = control.query_servo_angle(SERVO_ID)
        print(f"   Current position: {current_angle}°")
        
        # 3. Method 1: Simple angle command with time interval
        print("\n🎯 Method 1: Time-based movement")
        target1 = current_angle + 30
        print(f"   Moving to {target1}° over 3 seconds...")
        control.set_servo_angle(SERVO_ID, target1, interval=3000)  # 3 seconds
        time.sleep(4)  # Wait for completion
        new_angle = control.query_servo_angle(SERVO_ID)
        print(f"   Result: {current_angle}° → {new_angle}°")
        
        # 4. Method 2: Velocity-controlled movement
        print("\n🚀 Method 2: Velocity-controlled movement")
        target2 = current_angle - 40
        print(f"   Moving to {target2}° at 100°/sec...")
        control.set_servo_angle(SERVO_ID, target2, 
                              velocity=100.0,    # 100 degrees per second
                              t_acc=200,         # 200ms acceleration
                              t_dec=200)         # 200ms deceleration
        time.sleep(3)
        new_angle = control.query_servo_angle(SERVO_ID)
        print(f"   Result: {current_angle}° → {new_angle}°")
        
        # 5. Method 3: High-power movement (for stuck servos)
        print("\n💪 Method 3: High-power movement")
        target3 = current_angle + 20
        print(f"   Moving to {target3}° with high power...")
        control.set_servo_angle(SERVO_ID, target3, power=800)  # 800mW
        time.sleep(3)
        new_angle = control.query_servo_angle(SERVO_ID)
        print(f"   Result: {current_angle}° → {new_angle}°")
        
        # 6. Return to center
        print("\n🏠 Returning to center position...")
        control.set_servo_angle(SERVO_ID, 0, interval=2000)
        time.sleep(3)
        
        print("\n✅ Movement demonstration complete!")
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'uart' in locals():
            uart.close()

def explain_parameters():
    """Explain the key parameters for servo movement"""
    print("\n" + "="*60)
    print("SERVO MOVEMENT PARAMETERS EXPLAINED")
    print("="*60)
    
    print("\n🔧 ENABLING THE SERVO:")
    print("   control.set_damping(servo_id, 0)")
    print("   - MUST be called first!")
    print("   - 0 = disable damping = enable active movement")
    print("   - Non-zero = enable damping = servo becomes passive")
    
    print("\n⏱️  TIME-BASED MOVEMENT:")
    print("   control.set_servo_angle(id, angle, interval=3000)")
    print("   - interval: Time in milliseconds to complete movement")
    print("   - Good for: Smooth, predictable movements")
    print("   - Use when: You want consistent timing")
    
    print("\n🚀 VELOCITY-BASED MOVEMENT:")
    print("   control.set_servo_angle(id, angle, velocity=100.0, t_acc=200, t_dec=200)")
    print("   - velocity: Speed in degrees per second")
    print("   - t_acc: Acceleration time in milliseconds")
    print("   - t_dec: Deceleration time in milliseconds")
    print("   - Good for: Precise speed control")
    print("   - Use when: You need specific movement speeds")
    
    print("\n💪 POWER-BASED MOVEMENT:")
    print("   control.set_servo_angle(id, angle, power=800)")
    print("   - power: Force in milliwatts (typical range: 100-1000)")
    print("   - Good for: Overcoming resistance or stuck positions")
    print("   - Use when: Servo seems weak or unresponsive")
    
    print("\n📊 MONITORING MOVEMENT:")
    print("   current_angle = control.query_servo_angle(servo_id)")
    print("   - Always check position before and after movement")
    print("   - Verify the servo actually moved")
    print("   - Use for feedback and error detection")

if __name__ == "__main__":
    print("SERVO MOVEMENT DEMONSTRATION")
    print("="*40)
    
    # First explain the concepts
    explain_parameters()
    
    # Then demonstrate if user wants to run it
    print("\n" + "="*60)
    user_input = input("Run movement demonstration? (y/n): ")
    if user_input.lower().startswith('y'):
        demonstrate_servo_movement()
    else:
        print("Demonstration skipped. Run this script with 'y' to see actual movement.")
