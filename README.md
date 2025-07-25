# AdvXRobots - Advanced Robotic Control System

A modular robotic arm control system with face tracking capabilities for the AdvXRobots project.

## Features

- **Face Tracking**: Real-time face detection and tracking using computer vision
- **Servo Control**: Precise servo motor control for robotic arm movements
- **Visual Interface**: LCD display integration for status and feedback
- **Hardware Abstraction**: Clean interface layer for different hardware configurations
- **Kinematics**: Forward and inverse kinematics for arm positioning
- **Sequence Control**: Pre-programmed movement sequences and behaviors

## Project Structure

```
AdvXRobots/
├── .gitignore
├── README.md
├── requirements.txt
├── config/
│   └── arm_config.json          # Hardware configuration and parameters
├── docs/                        # Documentation and guides
├── models/                      # AI models for object detection
│   └── object_detection.tflite  # TensorFlow Lite object detection model
├── notebooks/                   # Jupyter notebooks for experimentation
├── src/                         # Main source code
│   ├── __init__.py
│   ├── main.py                  # Main application entry point
│   ├── hardware_interface.py    # Hardware abstraction layer
│   ├── vision_system.py         # Computer vision and face tracking
│   ├── display_manager.py       # LCD display management
│   ├── kinematics.py            # Robotic arm kinematics
│   ├── sequences.py             # Pre-programmed movement sequences
│   └── vendor/                  # Third-party SDKs and libraries
│       ├── seeed_arm_sdk/       # Seeed Studio arm SDK
│       ├── camera_sdk/          # Camera interface SDK
│       └── lcd_sdk/             # LCD display SDK
└── tests/                       # Unit tests and integration tests
```

## Hardware Requirements

- **Servo Motors**: 3-6 servo motors for arm joints
- **Camera**: USB or CSI camera for face tracking
- **Display**: LCD display for status and feedback
- **Controller**: Raspberry Pi or similar single-board computer
- **Communication**: UART/RS485 for servo communication

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-repo/EmoRobots.git
   cd EmoRobots/embedded/my_robot_arm
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Configure hardware settings:
   ```bash
   cp config/arm_config.json.example config/arm_config.json
   # Edit config/arm_config.json with your hardware settings
   ```

## Quick Start

1. **Test Hardware Connection**:
   ```bash
   python -m tests.test_hardware_interface
   ```

2. **Run Face Tracking Demo**:
   ```bash
   python src/main.py --mode face_tracking
   ```

3. **Execute Pre-programmed Sequence**:
   ```bash
   python src/main.py --mode sequence --sequence wave
   ```

## Configuration

The main configuration file is `config/arm_config.json`. Key settings include:

- **Servo Configuration**: IDs, ranges, initial positions
- **Camera Settings**: Resolution, framerate, calibration
- **Kinematics Parameters**: Link lengths, joint limits
- **Vision Parameters**: Detection thresholds, tracking parameters

## Usage Examples

### Basic Face Tracking
```python
from src.main import RobotArmController

controller = RobotArmController()
controller.start_face_tracking()
```

### Custom Movement Sequence
```python
from src.sequences import SequenceManager

seq_manager = SequenceManager(controller.hardware)
seq_manager.execute_sequence("custom_wave")
```

### Direct Servo Control
```python
from src.hardware_interface import HardwareInterface

hardware = HardwareInterface()
hardware.move_servo(1, 90, duration=1000)  # Move servo 1 to 90 degrees
```

## Development

### Running Tests
```bash
# Run all tests
python -m pytest tests/

# Run specific test
python -m pytest tests/test_vision_system.py
```

### Adding New Sequences
1. Create sequence definition in `config/arm_config.json`
2. Implement sequence logic in `src/sequences.py`
3. Add tests in `tests/test_sequences.py`

### Hardware Integration
1. Implement hardware-specific code in `src/vendor/`
2. Update hardware interface in `src/hardware_interface.py`
3. Update configuration in `config/arm_config.json`

## API Reference

### Main Classes

- **`RobotArmController`**: Main controller class
- **`HardwareInterface`**: Hardware abstraction layer
- **`VisionSystem`**: Computer vision and tracking
- **`DisplayManager`**: LCD display management
- **`KinematicsEngine`**: Forward/inverse kinematics
- **`SequenceManager`**: Movement sequence control

## Troubleshooting

### Common Issues

1. **Servo not responding**:
   - Check UART connections and baudrate
   - Verify servo ID configuration
   - Test power supply voltage

2. **Camera not detected**:
   - Check USB/CSI connection
   - Verify camera permissions
   - Test with `v4l2-ctl --list-devices` (Linux)

3. **Face tracking accuracy**:
   - Improve lighting conditions
   - Adjust detection thresholds
   - Calibrate camera parameters

### Debug Mode
Enable debug logging:
```bash
python src/main.py --debug --log-level DEBUG
```

## Contributing

1. Follow PEP 8 style guidelines
2. Add unit tests for new features
3. Update documentation for API changes
4. Test on actual hardware before submitting

## License

This project is part of the EmoRobots research project. Please refer to the main repository for licensing information.

## Support

For issues and questions:
- Check the [documentation](docs/)
- Search [existing issues](../../issues)
- Create a [new issue](../../issues/new) with detailed description

---

**Note**: This is part of the larger EmoRobots project focused on emotional robotics research and development.
