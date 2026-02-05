// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.config.ConfigLoader;
import frc.robot.config.RobotConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Load configuration once at startup
  private static final RobotConfig config = ConfigLoader.load();

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = config.operator.driverControllerPort;
    public static final int OPERATOR_CONTROLLER_PORT = config.operator.operatorControllerPort;
    public static final double DEADBAND = config.operator.deadband;
    public static final double DEADBAND_Y = config.operator.deadbandY;
  }

  public static class DriveConstants {
    // ==========================================
    // ROBOT HARDWARE CONFIGURATION
    // Loaded from src/main/deploy/robot-config.json
    // ==========================================
    public enum DrivetrainHardwareType {
      SPARK_MAX_NEO, // The 2024 Competition Robot (Brushless)
      TALON_SRX_CIM  // The Practice Robot (Brushed / Old Motors)
    }

    // Map config enum to local enum
    public static final DrivetrainHardwareType CURRENT_DRIVETRAIN_TYPE = 
        DrivetrainHardwareType.valueOf(config.drive.drivetrainType.name());

    public static final boolean HAS_DRIVE_ENCODERS = config.drive.hasDriveEncoders;

    // Motor CAN IDs
    public static final int FRONT_LEFT_MOTOR_ID = config.drive.frontLeft.id;
    public static final int REAR_LEFT_MOTOR_ID = config.drive.rearLeft.id;
    public static final int FRONT_RIGHT_MOTOR_ID = config.drive.frontRight.id;
    public static final int REAR_RIGHT_MOTOR_ID = config.drive.rearRight.id;

    // Motor inversions
    public static final boolean FRONT_LEFT_INVERTED = config.drive.frontLeft.inverted;
    public static final boolean REAR_LEFT_INVERTED = config.drive.rearLeft.inverted;
    public static final boolean FRONT_RIGHT_INVERTED = config.drive.frontRight.inverted;
    public static final boolean REAR_RIGHT_INVERTED = config.drive.rearRight.inverted;

    // Drive control
    public static final double RAMP_RATE = config.drive.rampRate;
    public static final double MAX_WHEEL_VELOCITY = config.drive.maxWheelVelocity; // RPM

    // PID constants
    public static final int PID_SLOT = config.drive.pid.slot;
    public static final double MOTOR_P = config.drive.pid.p;
    public static final double MOTOR_I = config.drive.pid.i;
    public static final double MOTOR_I_ZONE = config.drive.pid.iZone != null ? config.drive.pid.iZone : 0;
    public static final double MOTOR_D = config.drive.pid.d;
    public static final double MOTOR_FF = config.drive.pid.f;
    public static final double MOTOR_MAX_OUTPUT = config.drive.pid.maxOutput != null ? config.drive.pid.maxOutput : 1;
    public static final double MOTOR_MIN_OUTPUT = config.drive.pid.minOutput != null ? config.drive.pid.minOutput : -1;

    // SmartMotion constants
    public static final double MAX_VELOCITY = config.drive.smartMotion.maxVelocity; // RPM
    public static final double MAX_ACCELERATION = config.drive.smartMotion.maxAcceleration;
    public static final double MIN_VELOCITY = config.drive.smartMotion.minVelocity;
    public static final double ALLOWED_ERROR = config.drive.smartMotion.allowedError;

    // Position tolerance for autonomous
    public static final double POSITION_TOLERANCE = config.drive.positionTolerance; // Encoder revolutions
  }

  public static class PivotConstants {
    // ==========================================
    // CONFIGURATION
    // ==========================================
    public static final boolean PIVOT_ENABLED = config.pivot.enabled;

    // Motor CAN ID
    public static final int MOTOR_ID = config.pivot.motorId;

    // Position setpoints (encoder counts)
    public static final int SHOOTING_HIGH_POS = config.pivot.setpoints.shootingHigh;
    public static final int SHOOTING_LOW_POS = config.pivot.setpoints.shootingLow;
    public static final int INTAKE_POS = config.pivot.setpoints.intake;

    // Position tolerance
    public static final int POSITION_TOLERANCE = config.pivot.positionTolerance;

    // Motor configuration
    public static final boolean INVERTED = config.pivot.inverted;
    public static final boolean SENSOR_PHASE = config.pivot.sensorPhase;
    public static final double PEAK_OUTPUT_FORWARD = config.pivot.peakOutputForward;
    public static final double PEAK_OUTPUT_REVERSE = config.pivot.peakOutputReverse;

    // PID constants
    public static final int PID_SLOT = config.pivot.pid.slot;
    public static final int TIMEOUT_MS = config.pivot.pid.timeoutMs != null ? config.pivot.pid.timeoutMs : 30;
    public static final double MOTOR_F = config.pivot.pid.f;
    public static final double MOTOR_P = config.pivot.pid.p;
    public static final double MOTOR_I = config.pivot.pid.i;
    public static final double MOTOR_D = config.pivot.pid.d;

    // MotionMagic constants
    public static final int MOTION_CRUISE_VELOCITY = config.pivot.motionMagic.cruiseVelocity;
    public static final int MOTION_ACCELERATION = config.pivot.motionMagic.acceleration;
    public static final int MOTION_S_CURVE_STRENGTH = config.pivot.motionMagic.sCurveStrength;
  }

  public static class ShieldConstants {
    // ==========================================
    // CONFIGURATION
    // ==========================================
    public static final boolean SHIELD_ENABLED = config.shield.enabled;

    // Motor CAN ID
    public static final int MOTOR_ID = config.shield.motorId;

    // Position setpoints (encoder counts)
    public static final double DOWN_POS = config.shield.setpoints.down;
    public static final double MID_POS = config.shield.setpoints.mid;
    public static final double UP_POS = config.shield.setpoints.up;

    // Motor configuration
    public static final boolean INVERTED = config.shield.inverted;
    public static final boolean SENSOR_PHASE = config.shield.sensorPhase;
    public static final double PEAK_OUTPUT_FORWARD = config.shield.peakOutputForward;
    public static final double PEAK_OUTPUT_REVERSE = config.shield.peakOutputReverse;

    // PID constants
    public static final int PID_SLOT = config.shield.pid.slot;
    public static final int TIMEOUT_MS = config.shield.pid.timeoutMs != null ? config.shield.pid.timeoutMs : 30;
    public static final double MOTOR_F = config.shield.pid.f;
    public static final double MOTOR_P = config.shield.pid.p;
    public static final double MOTOR_I = config.shield.pid.i;
    public static final double MOTOR_D = config.shield.pid.d;

    // Initial position
    public static final double INITIAL_POSITION = config.shield.initialPosition;
  }

  public static class ShooterConstants {
    // ==========================================
    // CONFIGURATION
    // ==========================================
    public static final boolean SHOOTER_ENABLED = config.shooter.enabled;

    // Motor CAN IDs
    public static final int SHOOTER_MOTOR_ID = config.shooter.shooterMotorId;
    public static final int FEEDER_MOTOR_ID = config.shooter.feederMotorId;

    // Motor inversions
    public static final boolean SHOOTER_INVERTED = config.shooter.shooterInverted;
    public static final boolean FEEDER_INVERTED = config.shooter.feederInverted;

    // Speed setpoints
    public static final double SHOOTER_SPEED = config.shooter.shooterSpeed;
    public static final double SHOOTER_REVERSE_SPEED = config.shooter.shooterReverseSpeed;
    public static final double FEEDER_SPEED = config.shooter.feederSpeed;
    public static final double FEEDER_REVERSE_SPEED = config.shooter.feederReverseSpeed;
  }

    public static class AutoConstants {
      // Auto mode names
      public static final String DO_NOTHING = "Do Nothing";
      public static final String AUTO_1 = "Auto 1";
      public static final String AUTO_2 = "Auto 2";
      public static final String AUTO_3 = "Auto 3";
      public static final String AUTO_4 = "Auto 4";
      public static final String AUTO_5 = "Auto 5";
      public static final String AUTO_6 = "Auto 6";
  
      // Timing constants
      public static final double SHOOTER_SPINUP_TIME = config.auto.timing.shooterSpinupTime; // seconds     
      public static final double FEED_TIME = config.auto.timing.feedTime; // seconds
      public static final double WAIT_TIME = config.auto.timing.waitTime; // seconds for Auto 5/6
    }}
