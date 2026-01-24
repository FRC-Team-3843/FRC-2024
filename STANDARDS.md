# FRC-2024 Technical Standards

**Standards Version:** Updated to 2026 standards (APIs and architecture refactored)

**⚠️ IMPORTANT:** APIs change season to season. Always verify compatibility before copying to newer projects.

---

## Branch Guide

This repository has **two branches** with very different states:

### `main` Branch (CURRENT STANDARDS)
- **Status:** Fully refactored to modern standards
- **Architecture:** Command-based framework
- **Motor APIs:** `SparkMax` with `SparkMaxConfig`, `TalonFX` with Phoenix6
- **Patterns:** IO-layer abstraction, dependency injection, command factories
- **Use For:** Reference implementations of modern WPILib patterns

### `2024_Archive` Branch (DEPRECATED)
- **Status:** Original 2024 competition code (deprecated)
- **Architecture:** Monolithic TimedRobot (no command-based framework)
- **Motor APIs:** Old `CANSparkMax` API (removed in 2026)
- **Patterns:** Direct hardware calls, state machine autonomous
- **Use For:** Historical reference only - DO NOT copy code from this branch

**If you're looking at old/deprecated code, you're likely on the wrong branch!**

---

## Current Standards (main branch)

### Architecture

The refactored `main` branch uses modern WPILib command-based architecture:

**Entry Points:**
- `Main.java` - WPILib entry point
- `Robot.java` - Command scheduler runner
- `RobotContainer.java` - Subsystem instantiation and command bindings

**Subsystems** (hardware abstraction):
- `DriveSubsystem` - Mecanum drive control
- `PivotSubsystem` - Pivot mechanism
- `ShooterSubsystem` - Shooter mechanism
- `ShieldSubsystem` - Shield mechanism

**IO-Layer Pattern:**
- Each subsystem uses IO interfaces for hardware abstraction
- Example: `DriveIO`, `PivotIO`, `ShooterIO`, `ShieldIO`
- Real hardware implementations: `DriveIOSparkMax`, etc.
- Sim implementations: `DriveIOSim`, etc.

**Commands:**
- Command factories in subsystems (preferred pattern)
- Autonomous commands built using PathPlanner and command compositions
- `CommandXboxController` for driver input with trigger API

**Configuration:**
- `Constants.java` - All configuration centralized
- CAN IDs, sensor configs, PID constants, positions defined here

### Motor Control APIs

**SparkMax (NEO motors):**
```java
SparkMax motor = new SparkMax(canId, MotorType.kBrushless);
SparkMaxConfig config = new SparkMaxConfig();
config.inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40);
motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
```

**TalonFX (Kraken/Falcon motors - if used):**
```java
TalonFX motor = new TalonFX(canId);
TalonFXConfiguration config = new TalonFXConfiguration();
config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
motor.getConfigurator().apply(config);
```

### Controller Input

Uses `CommandXboxController` with trigger API:
```java
CommandXboxController driver = new CommandXboxController(0);
driver.a().onTrue(subsystem.someCommand());
driver.leftTrigger().whileTrue(subsystem.anotherCommand());
```

---

## Copying to Newer Projects

**CRITICAL:** APIs change season to season. Before copying ANY code to a newer project:

1. **Verify API compatibility** - Check that all APIs still exist and work the same way
2. **Check deprecation warnings** - WPILib frequently deprecates old patterns
3. **Review architecture patterns** - Ensure the pattern is still recommended
4. **Test thoroughly** - Don't assume code that worked in 2024 will work unchanged

**What's Safe to Reference:**
- High-level architecture patterns (subsystems, commands, IO-layer)
- Game-specific strategy and autonomous routines (adapt to new APIs)
- Mecanum drive kinematics and control logic (verify APIs)

**What Requires Verification:**
- ALL motor control code (APIs change frequently)
- Controller binding patterns (verify trigger API is still current)
- Vendor library usage (REVLib, Phoenix6, etc.)

---

## Build Commands

From `FRC-2024/2024Robot/`:
```bash
./gradlew build          # Build the project
./gradlew deploy         # Deploy to robot
./gradlew test           # Run unit tests
./gradlew simulateJava   # Run robot simulation
```

**Team Number:** 3843

---

## 2024 Season Context

**Team:** FRC 3843
**Robot:** 2024Robot (mecanum drive)
**Game:** CRESCENDO (2024 FRC game)

**Key Features:**
- Mecanum drive (4-wheel omnidirectional)
- Pivot mechanism for game piece manipulation
- Shooter mechanism
- Shield mechanism

**Drive Configuration:**
- 4 NEO motors (one per wheel)
- Mecanum wheels for omnidirectional movement
- Field-centric and robot-centric drive modes

**Why This Matters:**
- Understanding the 2024 game helps contextualize code decisions
- Mecanum kinematics are game-agnostic and can be referenced
- Game-specific mechanisms show how to structure multi-subsystem robots

---

## Repository Status

**Last Updated:** 2026-01-23
**Status:** REFERENCE - Modern standards applied to 2024 robot
**Use For:** Learning modern WPILib patterns, understanding mecanum drive, seeing IO-layer abstraction in practice
**Maintained:** `main` branch kept up-to-date with current standards; `2024_Archive` branch frozen

---

**For the latest standards:** See the newest season's `STANDARDS.md` file (currently `FRC-2026\STANDARDS.md`)
