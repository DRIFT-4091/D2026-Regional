# Practice Drivetrain

A WPILib Java robot project featuring a swerve drivetrain with vision-assisted control using Limelight.

## Overview

This is an FRC robot code project built with WPILib 2026.2.1 that implements a command-based swerve drive system using CTRE Phoenix 6 libraries. The robot features advanced vision-assisted driving capabilities for precise targeting and positioning.

## Key Features

- **Swerve Drivetrain**: Full command-based swerve drive implementation using Phoenix 6 Tuner X
- **Field-Centric Drive**: Allows intuitive driving from the driver's perspective
- **Vision Targeting**: Limelight integration for automated target tracking and positioning
  - **Aim Assist Mode**: Auto-rotates robot to center on target while maintaining manual drive control
  - **Drive-to-Target Mode**: Automatically drives to optimal shooting/intake distance while aiming
- **SysId Integration**: Built-in system identification routines for tuning drive and steer motors
- **Auto Replay**: CTRE Hoot auto replay for logging and replaying timestamp and joystick data

## Controls (PS4 Controller)

### Basic Drive
- **Left Stick**: Forward/backward and strafe movement
- **Right Stick**: Rotation control
- **L1**: Reset field-centric heading

### Special Functions
- **Cross (X)**: Brake mode (lock wheels in X formation)
- **Circle (○)**: Point wheels in specified direction
- **R1**: Aim assist - auto-rotate to center target (manual drive maintained)
- **R2**: Drive to target - auto-drive to optimal distance and auto-aim

### SysId Characterization
- **Share + Triangle**: SysId dynamic forward
- **Share + Square**: SysId dynamic reverse
- **Options + Triangle**: SysId quasistatic forward
- **Options + Square**: SysId quasistatic reverse

## Project Structure

- `src/main/java/frc/robot/`
  - `Robot.java` - Main robot class with periodic methods
  - `RobotContainer.java` - Command bindings and robot configuration
  - `Limelight.java` - Limelight vision utility class
  - `Telemetry.java` - Telemetry logging for drivetrain
  - `subsystems/CommandSwerveDrivetrain.java` - Swerve drivetrain subsystem
  - `generated/TunerConstants.java` - Auto-generated tuner constants

## Build & Deploy

This project uses Gradle for building and deploying:

```bash
# Build the project
./gradlew build

# Deploy to robot
./gradlew deploy

# Run simulation
./gradlew simulateJava
```

## Dependencies

- WPILib 2026.2.1
- CTRE Phoenix 6 (version 26.1.0)
- Java 17

## License

This project is based on WPILib and follows the WPILib BSD license. See `WPILib-License.md` for details.
