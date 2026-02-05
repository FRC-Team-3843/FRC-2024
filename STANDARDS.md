# FRC-2024 Technical Standards

> **This repository follows FRC-2026 standards.**
> See `C:\GitHub\FRC-2026\STANDARDS.md` for the complete technical reference.
>
> **Documentation Guide:**
> - **This file (STANDARDS):** Repository-specific context and status
> - **README.md:** Project overview and quick start
> - **NOTES.md:** Setup procedures, tuning values, troubleshooting
> - **FRC-2026\STANDARDS.md:** Complete coding standards and API reference

---

## Repository-Specific Context

### Branch Guide

This repository has **two branches** with very different states:

#### `main` Branch (CURRENT STANDARDS)
- **Status:** Fully refactored to modern standards
- **Architecture:** Command-based framework with IO-layer abstraction
- **Motor APIs:** `SparkMax` with `SparkMaxConfig`, `TalonFX` with Phoenix6
- **Patterns:** Dependency injection, command factories
- **Use For:** Reference implementations of modern WPILib patterns

#### `2024_Archive` Branch (DEPRECATED)
- **Status:** Original 2024 competition code (deprecated)
- **Architecture:** Monolithic TimedRobot (no command-based framework)
- **Motor APIs:** Old `CANSparkMax` API (removed in 2026)
- **Patterns:** Direct hardware calls, state machine autonomous
- **Use For:** Historical reference only - DO NOT copy code from this branch

**If you're looking at old/deprecated code, you're likely on the wrong branch!**

---

## Current State (main branch)

### Architecture Overview

The refactored `main` branch uses modern WPILib command-based architecture with IO-layer abstraction:

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

---

## What to Reference from This Repo

### ✅ Safe to Reference:

- **IO-Layer Pattern:** Hardware abstraction interfaces for subsystems
- **Mecanum Kinematics:** Drive control logic for omnidirectional movement
- **Command Factories:** Examples of inline command factory methods
- **Architecture Structure:** Clean separation of Robot/RobotContainer/Subsystems

### ⚠️ Use with Caution:

- **Game-Specific Logic:** Adapt to new game requirements
- **Motor Configurations:** Verify APIs are still current

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

**Last Updated:** 2026-01-26
**Status:** REFERENCE - Modern standards applied to 2024 robot
**Use For:** Learning modern WPILib patterns, understanding mecanum drive, seeing IO-layer abstraction in practice
