# FRC-2024 - Crescendo

FRC Team 3843 - 2024 Season

## Overview
Team 3843's robot for the 2024 Crescendo game. This robot features a mecanum drive system with flexible hardware support, allowing the same codebase to run on both competition and practice robots. Key capabilities include note shooting, pivoting mechanism for angle adjustment, and autonomous routines.

## Hardware

### Drivetrain
- **Type:** Mecanum Drive (4-wheel omnidirectional)
- **Dual Hardware Support:**
  - **Competition Bot:** NEO brushless motors via SparkMax controllers
  - **Practice Bot:** CIM brushed motors via TalonSRX controllers
- **Configuration:** Switchable via Constants.java (see below)

### Mechanisms
- **Pivot:** Angle adjustment for shooting (TalonSRX)
- **Shooter:** Dual-motor system (shooter + feeder)
- **Shield:** Optional defensive mechanism (can be disabled)

## Software Stack
- WPILib 2024
- Command-Based Architecture with Hardware Abstraction
- REVLib (SparkMax/NEO)
- Phoenix5 (TalonSRX/CIM)

## CAN ID Assignments

| Device | CAN ID | Description |
|--------|--------|-------------|
| Front Left Drive | 1 | Mecanum wheel motor |
| Rear Left Drive | 2 | Mecanum wheel motor |
| Front Right Drive | 3 | Mecanum wheel motor |
| Rear Right Drive | 4 | Mecanum wheel motor |
| Pivot Motor | 5 | Shooter angle adjustment |
| Shield Motor | 6 | Defensive shield (optional) |
| Feeder Motor | 7 | Note feeding system |
| Shooter Motor | 8 | Main shooter flywheel |

## Subsystems
- **Drive:** Mecanum drive with field-centric control and hardware abstraction layer
- **Pivot:** Position-controlled mechanism for shooter angle adjustment
- **Shooter:** Dual-motor note shooting system
- **Shield:** Optional defensive mechanism

## Hardware Abstraction Layer
This codebase uses a professional IO-layer pattern to support multiple hardware configurations:
- `DriveIO` interface defines standard drive operations
- `DriveIOSparkMax` implements NEO motor support
- `DriveIOTalon` implements CIM motor support

**Switching Hardware:** Edit `Constants.java`:
```java
// For Competition Bot (NEO motors)
CURRENT_DRIVETRAIN_TYPE = DrivetrainHardwareType.SPARK_MAX_NEO;
HAS_DRIVE_ENCODERS = true;

// For Practice Bot (CIM motors)
CURRENT_DRIVETRAIN_TYPE = DrivetrainHardwareType.TALON_SRX_CIM;
HAS_DRIVE_ENCODERS = false; // or true if encoders added
```

## Building and Deploying
```bash
cd 2024Robot
./gradlew build
./gradlew deploy
```

## Additional Documentation
- [Drivetrain Abstraction Explanation](2024Robot/docs/DRIVETRAIN_EXPLANATION.md) - Detailed guide for students on the hardware abstraction pattern

## Competition Season
The 2024 Crescendo game focused on scoring notes in various targets including the Speaker and Amp. This robot was designed for versatile shooting capabilities with adjustable angles and reliable autonomous performance.
