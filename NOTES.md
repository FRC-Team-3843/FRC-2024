# Working Notes - FRC-2024

> **Documentation Guide:**
> - **This file (NOTES):** Setup, tuning, troubleshooting, TODOs
> - **README.md:** Project overview and quick start
> - **STANDARDS.md:** Coding standards (references FRC-2026\STANDARDS.md)

## CAN Bus Assignments

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

## Pre-Flight Checklist
- [ ] Verify hardware configuration in Constants.java matches current robot
- [ ] Check CAN IDs match physical wiring
- [ ] Test motor directions after deployment
- [ ] Verify encoder functionality (if equipped)

## Tuning Values

### Drive System
- Max Wheel Velocity: 5000 RPM
- Ramp Rate: 0.2
- PID: P=0.0001, I=0.000001, D=0, FF=0.000156

### Pivot Mechanism
- Shooting High: 8000 counts
- Shooting Low: 73000 counts
- Intake: 160000 counts
- PID: P=2.0, I=0.0008, D=2.0, F=0.02325

## Controller Mappings
- Driver Controller: Port 0
- Operator Controller: Port 1
- Deadband: 0.12 (Y-axis: 0.24)

## Hardware Configurations

### Current Practice Bot Setup
- Drivetrain: TalonSRX/CIM
- Encoders: Disabled
- Note: Autonomous drive commands will skip if encoders disabled

### Competition Bot Setup
- Drivetrain: SparkMax/NEO
- Encoders: Built-in to NEO motors
- Full autonomous functionality

## TODOs
- [ ] Season complete - archive only

## Lessons Learned
- Hardware abstraction layer enabled seamless switching between robots
- Dual robot support saved development time during build season
- IO-layer pattern is professional approach worth maintaining
