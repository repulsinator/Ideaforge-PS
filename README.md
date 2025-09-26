# Fault-Tolerant Quadcopter Control System

A comprehensive solution for detecting motor failures and implementing fault-tolerant control in quadcopter UAVs, developed for the Inter IIT Tech Meet 13.0 ideaForge Aerial Robotics Challenge.

## Overview

This project addresses the critical challenge of maintaining quadcopter stability and control during single motor failures. Our solution combines advanced machine learning-based failure detection with intelligent control algorithms to enable safe operation even when compromised.

### Key Capabilities
- **Implicit Motor Failure Detection**: Real-time detection without dedicated sensors
- **Controlled Emergency Landing**: Stable spinning descent with three motors
- **Stable Hovering**: Maintain altitude and position despite motor loss
- **Navigation & Recovery**: Basic flight control and safe return-to-home functionality

## System Architecture

### 1. Motor Failure Detection

#### Threshold-Based Detection
- **Real-time monitoring** of IMU sensor data (gyroscope, accelerometer, magnetometer)
- **Anomaly detection** through statistical threshold analysis
- **Sensitivity optimization** to balance detection speed and noise resilience
- **Detection latency**: 0.48-0.63 seconds (MAVLink), optimized for direct PX4 integration

#### Machine Learning Approach (MLP)
- **Multi-Layer Perceptron** trained on comprehensive flight data
- **Classification accuracy**: 84% across 5 motor states (Normal, Motor 1-4 failure)
- **Real-time inference** integrated into PX4 Control Allocator
- **Robust performance** under various flight conditions and noise

```
Model Architecture:
├── Input Layer (6 neurons): ax, ay, az, gx, gy, gz
├── Hidden Layer 1 (128 neurons, ReLU)
├── Hidden Layer 2 (64 neurons, ReLU)
└── Output Layer (5 neurons, Softmax): Motor states
```

### 2. Fault-Tolerant Control

#### Nonlinear Model Predictive Control (NMPC)
- **Real-time optimization** of control inputs over prediction horizon
- **Constraint handling** for actuator limits and safety bounds
- **Adaptive control** redistribution among remaining motors
- **Integration** with CasADi and Acados for embedded deployment

#### Reinforcement Learning Control
- **Adaptive policy learning** through Proximal Policy Optimization (PPO)
- **Markov Decision Process** formulation for continuous action spaces
- **Neural network controllers** for both 3-motor and 4-motor configurations
- **Simulation training** in Gazebo with realistic wind disturbances

### 3. PX4 Integration

#### System Modifications
- **Custom Control Allocator** with integrated failure detection
- **Disabled default failsafes** to enable custom recovery algorithms
- **Optimized configuration** for reduced mass and improved thrust efficiency
- **Real-time data pipeline** through uORB topics at 200 Hz

#### Communication Architecture
```
IMU Sensors → Control Allocator → Failure Detection → Recovery Controller → Motor Commands
     ↓              ↓                    ↓                    ↓               ↓
  200 Hz        uORB Topics        MLP/Threshold        NMPC/RL        Actuator Output
```

## Installation & Setup

### Prerequisites
```bash
# System requirements
- Ubuntu 20.04 LTS
- ROS 2 Foxy
- PX4 v1.13+
- Gazebo 11
- Python 3.8+

# Dependencies
- CasADi
- Acados
- MAVSDK
- TensorFlow/PyTorch
```

### Quick Start
```bash
# Clone repository
git clone https://github.com/your-team/fault-tolerant-quadcopter.git
cd fault-tolerant-quadcopter

# Install dependencies
./install_dependencies.sh

# Build PX4 with custom modifications
cd px4-firmware
make px4_sitl gazebo

# Launch simulation
./launch_simulation.sh
```

## Dataset & Training

### Data Collection
- **Comprehensive scenarios**: Hovering, directional flight, various failure modes
- **Environmental conditions**: Normal and windy (up to 20 m/s) conditions  
- **Flight parameters**: ±30° roll/pitch, π rad/sec yaw rates
- **Sensor frequency**: 200 Hz IMU data collection
- **Dataset size**: 75 datasets per motor failure type

### Model Training
```bash
# Train MLP failure detection model
python train_mlp_detector.py --data-path ./datasets --epochs 100

# Train RL control policy
python train_rl_controller.py --algorithm ppo --timesteps 1000000

# Generate NMPC solver code
python generate_nmpc_solver.py --export-path ./px4-firmware/src/modules
```

## Performance Results

### Detection Accuracy
| Method | Accuracy | Precision | Recall | F1-Score |
|--------|----------|-----------|--------|----------|
| Threshold | 95%+ | - | - | - |
| MLP | 84% | 0.84 | 0.84 | 0.83 |

### Motor-Specific Performance (MLP)
| Motor State | Precision | Recall | F1-Score | Support |
|-------------|-----------|--------|----------|---------|
| Normal (0) | 0.99 | 1.00 | 0.99 | 13,269 |
| Motor 1 | 0.85 | 0.85 | 0.85 | 3,988 |
| Motor 2 | 0.88 | 0.86 | 0.87 | 3,909 |
| Motor 3 | 0.81 | 0.78 | 0.80 | 3,950 |
| Motor 4 | 0.63 | 0.35 | 0.45 | 4,025 |

## Simulation Environment

### Gazebo Windy World
- **Realistic physics simulation** with environmental disturbances
- **Dynamic wind modeling** with configurable parameters
- **Failure injection** capabilities for comprehensive testing
- **Real-time performance monitoring** and data logging

### Test Scenarios
- Motor failure during hover
- Motor failure during aggressive maneuvers
- Recovery in windy conditions
- Multi-waypoint navigation with failures

## Future Development

### Planned Enhancements
- **Advanced RL algorithms**: Soft Actor-Critic (SAC) implementation
- **Real-time NMPC optimization**: GPU acceleration and adaptive horizons
- **Ensemble learning**: Combined detection methods for improved robustness
- **Hardware validation**: Real-world flight testing and validation

### Known Limitations
- NMPC solver stability under extreme conditions
- MLP performance variation across motor types (especially Motor 4)
- Computational constraints in embedded deployment

## Contributing

We welcome contributions to improve the fault-tolerant control system. Please follow our contribution guidelines:

1. Fork the repository
2. Create feature branches for new capabilities
3. Implement comprehensive tests for new features
4. Submit pull requests with detailed descriptions

### Development Areas
- Enhanced failure detection algorithms
- Advanced control strategies
- Real-world testing and validation
- Performance optimization

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **ideaForge** for providing the challenge framework
- **Inter IIT Tech Meet 13.0** organizing committee
- **PX4 Development Team** for the open-source autopilot platform
- **Research contributions** from cited academic papers


