# Hardware Setup Guide

This guide covers the hardware setup and configuration for the EmoRobots robot arm system.

## Hardware Components

### Required Components
- **Servo Motors**: 3-6 servo motors (recommended: high-torque digital servos)
- **Microcontroller**: Raspberry Pi 4 or similar single-board computer
- **Camera**: USB webcam or Raspberry Pi Camera Module
- **Display**: Small LCD or OLED display for status (optional)
- **Power Supply**: 12V power supply capable of handling servo load
- **Servo Driver**: UART/RS485 servo controller board

### Recommended Hardware
- **Servos**: Feetech SCS series or similar smart servos with UART control
- **Camera**: Logitech C920 or Raspberry Pi Camera Module v2
- **Display**: 128x64 OLED display with I2C interface
- **Power**: 12V 5A switching power supply

## Wiring Connections

### Servo Connections
```
Servo Controller Board ← UART → Raspberry Pi
GND                    ←      → GND (Pin 6)
VCC (3.3V)            ←      → 3.3V (Pin 1)  
TX                     ←      → GPIO 14 (Pin 8)
RX                     ←      → GPIO 15 (Pin 10)
```

### Camera Connection
- **USB Camera**: Connect to any available USB port
- **Pi Camera**: Connect to CSI camera port on Raspberry Pi

### Display Connection (I2C)
```
OLED Display  ← I2C → Raspberry Pi
GND           ←     → GND (Pin 9)
VCC           ←     → 3.3V (Pin 17)
SDA           ←     → GPIO 2 (Pin 3)
SCL           ←     → GPIO 3 (Pin 5)
```

## Configuration

### Enable Required Interfaces
```bash
sudo raspi-config
# Enable: Camera, I2C, Serial (without login shell)
```

### Install System Dependencies
```bash
sudo apt update
sudo apt install python3-pip python3-opencv python3-serial
pip3 install -r requirements.txt
```

### Configure Servo IDs
Each servo should be configured with a unique ID:
- Servo 0: Base rotation
- Servo 1: Vertical tilt (face tracking)
- Servo 2: Horizontal pan (face tracking)  
- Servo 3: Arm extension
- Servo 4: Gripper rotation
- Servo 5: Gripper clamp

### Update Configuration File
Edit `config/arm_config.json`:
```json
{
  "hardware": {
    "servos": {
      "communication": {
        "port": "/dev/ttyS0",
        "baudrate": 1000000
      }
    },
    "camera": {
      "device_id": 0
    }
  }
}
```

## Testing Hardware

### Test Servo Communication
```bash
python3 -c "
from src.hardware_interface import HardwareInterface
config = {'servos': {'communication': {'port': '/dev/ttyS0', 'baudrate': 1000000}}}
hw = HardwareInterface(config)
print('Connected:', hw.initialize())
"
```

### Test Camera
```bash
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
print('Camera working:', ret and frame is not None)
cap.release()
"
```

### Test Display
```bash
python3 -c "
from src.display_manager import DisplayManager
config = {'type': 'LCD', 'interface': 'I2C'}
display = DisplayManager(config)
print('Display initialized:', display.initialize())
"
```

## Troubleshooting

### Servo Issues
- **No response**: Check wiring, power supply, and baud rate
- **Erratic movement**: Check power supply capacity and servo IDs
- **Communication errors**: Verify UART settings and cable connections

### Camera Issues  
- **Not detected**: Check USB connection or camera module connection
- **Poor quality**: Adjust lighting conditions and camera settings
- **Permissions**: Add user to video group: `sudo usermod -a -G video $USER`

### Display Issues
- **Blank display**: Check I2C wiring and address configuration
- **Wrong content**: Verify display resolution settings
- **I2C errors**: Enable I2C interface and check device address

### General Tips
- Always check power supply voltage and current capacity
- Verify all ground connections
- Use proper cable shielding for servo communication
- Check system logs: `journalctl -f` or `dmesg`

## Safety Considerations

- **Power**: Always disconnect power when making wiring changes
- **Movement**: Ensure clear workspace before running servo movements
- **Emergency Stop**: Test emergency stop functionality before operation
- **Limits**: Configure proper angle limits to prevent mechanical damage
