# Robot Configuration Guide

This robot project uses a JSON-based configuration system to allow rapid tuning of constants without recompiling the code.

## Configuration File

The configuration file is located at:
`src/main/deploy/robot-config.json`

This file is deployed to the RoboRIO automatically when code is deployed.

## How to Edit

1.  Open `src/main/deploy/robot-config.json` in VS Code.
2.  Edit the values as needed.
3.  **Deploy the code** (or just the deploy files) to the robot for changes to take effect.
    *   *Note: Currently, a full code deploy is required for the robot to read the file on startup.*

### Schema Validation
A JSON Schema (`robot-config.schema.json`) is included in the same directory.
*   **VS Code**: Will automatically provide autocomplete and validation (red squiggles for errors) because of the `"$schema"` line at the top of the config file.
*   **Hover**: Hover over properties in the JSON file to see descriptions of what they control.

## Sections

### Operator
Controls controller ports and joystick deadbands.
*   `driverControllerPort`: USB port for the driver.
*   `deadband`: Minimum joystick input to register movement.

### Drive
Configures the drivetrain hardware and PID loops.
*   `drivetrainType`: `SPARK_MAX_NEO` (Comp Bot) or `TALON_SRX_CIM` (Practice Bot).
*   `pid`: Tuning values for the drive motors.

### Pivot, Shield, Shooter
Subsystem-specific configurations including:
*   `motorId`: CAN ID of the motor.
*   `setpoints`: Target positions (encoder ticks) or speeds.
*   `pid`: Control loop constants (P, I, D, F).

### Auto
*   `timing`: Duration for various auto actions (spinup, feed, wait).
*   *Note: Auto Modes are defined in code (Constants.java), not in this JSON.*

## Troubleshooting

*   **Robot Crash on Boot**: If the JSON is invalid (syntax error) or missing required fields, the robot code will crash with a `RuntimeException`. Check the Driver Station logs.
*   **Changes Not Applying**: Ensure you have deployed the latest code/files to the robot.
