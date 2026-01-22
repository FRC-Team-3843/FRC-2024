# Drivetrain Refactor: How It Works
**For FRC Team 3843 Students**

## The Goal
We needed a way to run our robot code on **two different versions of the robot**:
1.  **The "Good" Version:** Uses **NEO** motors (SparkMax) and has encoders (sensors that measure speed/distance).
2.  **The "Practice" Version:** Uses **CIM** motors (TalonSRX) and currently has **no encoders**.

In the past, we might have created two different code branches or commented out large chunks of code. That is messy and error-prone. Instead, we used a professional software pattern called **Hardware Abstraction**.

---

## The Concept: The "Universal Remote"
Imagine you have a TV and a DVD player. They work differently, but you want to do the same things to both: "Turn On", "Turn Off", "Volume Up".

*   **The Interface (`DriveIO`):** This is the buttons on your universal remote. It defines *what* you can do (`setVolts`, `getSpeed`), but not *how* it happens.
*   **The Implementations (`DriveIOTalon`, `DriveIOSparkMax`):** These are the specific signals sent to the TV or DVD player.
    *   `DriveIOSparkMax` talks "REV SparkMax Language".
    *   `DriveIOTalon` talks "CTRE Talon Language".
*   **The Subsystem (`DriveSubsystem`):** This is *You* holding the remote. You just press "Volume Up", and you don't care *how* the remote sends the signal, as long as it works.

---

## File Walkthrough

### 1. `DriveIO.java` (The Contract)
This file is an **Interface**. It lists the methods that *must* exist, but has no code inside them.
*   `setVoltages(...)`: "Apply power to motors."
*   `getVelocities()`: "Tell me how fast we are going."

### 2. `DriveIOSparkMax.java` (The NEO Version)
This file handles the **REV SparkMax** motors.
*   It initializes `SparkMax` objects.
*   It reads directly from the built-in NEO encoders.
*   This is exactly what our code looked like before the change.

### 3. `DriveIOTalon.java` (The CIM Version)
This file handles the **CTRE TalonSRX** motors.
*   It initializes `WPI_TalonSRX` objects.
*   **The "No Encoder" Trick:** We added a check (`if HAS_DRIVE_ENCODERS`).
    *   If we have encoders, it calculates speed normally.
    *   **If we don't have encoders:** It "fakes" it! When the robot asks to go 50% speed, it just sends 50% voltage. It reports the speed as "0" so the code doesn't crash, but the robot still moves.

### 4. `DriveSubsystem.java` (The Brain)
This file controls the robot logic (Mecanum math, field-centric driving).
*   **Key Change:** It no longer creates `SparkMax` or `TalonSRX` objects directly.
*   **Startup Logic:** When the robot turns on, it checks `Constants.java`. If we selected `TALON_SRX_CIM`, it creates the Talon version. If we selected `SPARK_MAX_NEO`, it creates the SparkMax version.

---

## How to Change Hardware

Go to `src/main/java/frc/robot/Constants.java` and look for `DriveConstants`.

### Scenario A: Running on the CIM Practice Bot (Current Setup)
```java
public static final DrivetrainHardwareType CURRENT_DRIVETRAIN_TYPE = DrivetrainHardwareType.TALON_SRX_CIM;
public static final boolean HAS_DRIVE_ENCODERS = false;
```
*   **Effect:** The robot uses Talons. Autonomous commands that require distance (like "Drive 1 meter") will **skip instantly** because we can't measure distance. Manual driving works fine.

### Scenario B: We Added Encoders to the Practice Bot!
1.  Plug in the encoders.
2.  Change the code:
```java
public static final boolean HAS_DRIVE_ENCODERS = true;
```
*   **Effect:** Now "Drive 1 meter" autonomous routines will work!

### Scenario C: Switching to the NEO Competition Bot
```java
public static final DrivetrainHardwareType CURRENT_DRIVETRAIN_TYPE = DrivetrainHardwareType.SPARK_MAX_NEO;
// Encoders are built-in to NEOs, so this flag might be ignored or should be true
public static final boolean HAS_DRIVE_ENCODERS = true;
```

---

## Troubleshooting "Red Lines" in VS Code
If VS Code shows errors like `DriveIO cannot be resolved`, but the code builds fine:
1.  Press `Ctrl + Shift + P`.
2.  Type **"Java: Clean Java Language Server Workspace"**.
3.  Click it and confirm.
4.  Reload the window.
This forces VS Code to re-scan the new files we created.
