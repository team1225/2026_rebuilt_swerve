# FRC 2026 Sweve Base Test Robot

This repository contains the code for our 2026 FRC test robot, designed for the FIRST Robotics Competition.

## Hardware Specifications

### Drivetrain
- Swerve Drive Specialties MK4i modules
- NEO 500 motors for both drive and turning
- Gear ratio: L2 6.75:1
- Maximum speed: TBD

To set the zeros of the drivetrain align the wheels so the bolt head faces the port side (left if standing behind the robot).  Use the Rev Hardware Client to set the zero on of the absolute encoders remembering to persist parameters.

### Intake
- 4 belts
- NEO brushless motor with 3:1 gearbox
- 7 inches from ground to the center of intake wheals
- 3 inch complient wheals 3 times

### Sensors
- CTRE Pigeon 2.0 IMU for robot orientation and field-relative driving

## Software

### Development Environment
- Written in Java
- Built with Gradle
- WPILib 2026 (required)
- REV Hardware Client (for NEO motor configuration)
- Phoenix Tuner X (for Pigeon 2.0 configuration) 

### Project Structure
```
src/main/java/frc/robot/
├── commands/        # Command classes for robot actions
├── subsystems/      # Subsystem classes for major robot components
├── constants/       # Configuration constants and ports
└── Robot.java       # Main robot class
```

### Key Features
- Field-oriented swerve drive control
- Autonomous Commands:
  - None currently
- [Add other major features]
- [Add autonomous routines if developed]

### Controls

#### Driver Controls (Xbox Controller)
- Left Stick: Drive robot (Y axis for forward/backward, X axis for strafing)
- Right Stick: Rotate robot (X axis)
- A button: Wheals turn to X shape (useful for not getting pushed)
- B button: Intake operation (will be moved)

## Getting Started

### Prerequisites
1. Install WPILib 2026
2. Install REV Hardware Client
3. Install Phoenix Tuner X
4. [Any other required software]

### Installation
1. Clone the repository:
```bash
git clone https://github.com/team1225/2026-swerve-drive.git
```

2. Open the project in WPILib VS Code
3. Build the project:
```bash
./gradlew build
```

### Development Workflow
1. Create a new branch for your feature
2. Make your changes
3. Test on the robot
   - For autonomous testing:
     - Use SmartDashboard to select desired autonomous routine
     - Ensure ample clear space in front of robot when testing Simple Drive
     - Always be ready to disable the robot if needed
4. Create a pull request
5. Get code review
6. Merge after approval

## Contributing
To contribute to this project, come join us at Team 1225.

## Team
Team 1225 - The Gorillas
Henderson County Public Schools High School Robotics

Our team represents Henderson County Public Schools in North Carolina, competing in the FIRST Robotics Competition. The Gorillas have been inspiring students and promoting STEM education through competitive robotics since our founding.

## License
[Add license information]