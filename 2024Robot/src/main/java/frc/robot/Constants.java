// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DEADBAND = 0.12;
    public static final double DEADBAND_Y = 0.24; // Y-axis uses double deadband
  }

  public static class DriveConstants {
    // ==========================================
    // ROBOT HARDWARE CONFIGURATION
    // Change these lines to switch between the Practice Bot and Comp Bot!
    // ==========================================
    public enum DrivetrainHardwareType {
      SPARK_MAX_NEO, // The 2024 Competition Robot (Brushless)
      TALON_SRX_CIM  // The Practice Robot (Brushed / Old Motors)
    }

    // CHANGE THIS to switch bots:
    public static final DrivetrainHardwareType CURRENT_DRIVETRAIN_TYPE = DrivetrainHardwareType.TALON_SRX_CIM;
    
    // CHANGE THIS if you add/remove encoders (sensors) on the practice bot:
    // false = No sensors (Auto will skip move commands, Manual driving works)
    // true  = Sensors active (Auto works normally)
    public static final boolean HAS_DRIVE_ENCODERS = false;

    // Motor CAN IDs
    public static final int FRONT_LEFT_MOTOR_ID = 1;
    public static final int REAR_LEFT_MOTOR_ID = 2;
    public static final int FRONT_RIGHT_MOTOR_ID = 3;
    public static final int REAR_RIGHT_MOTOR_ID = 4;

    // Motor inversions
    public static final boolean FRONT_LEFT_INVERTED = true;
    public static final boolean REAR_LEFT_INVERTED = true;
    public static final boolean FRONT_RIGHT_INVERTED = false;
    public static final boolean REAR_RIGHT_INVERTED = false;

    // Drive control
    public static final double RAMP_RATE = 0.2;
    public static final double MAX_WHEEL_VELOCITY = 5000; // RPM

    // PID constants
    public static final int PID_SLOT = 0;
    public static final double MOTOR_P = 0.0001;
    public static final double MOTOR_I = 0.000001;
    public static final double MOTOR_I_ZONE = 0;
    public static final double MOTOR_D = 0;
    public static final double MOTOR_FF = 0.000156;
    public static final double MOTOR_MAX_OUTPUT = 1;
    public static final double MOTOR_MIN_OUTPUT = -1;

    // SmartMotion constants
    public static final double MAX_VELOCITY = 5000; // RPM
    public static final double MAX_ACCELERATION = 8000;
    public static final double MIN_VELOCITY = 0;
    public static final double ALLOWED_ERROR = 0;

    // Position tolerance for autonomous
    public static final double POSITION_TOLERANCE = 0.5; // Encoder revolutions
  }

  public static class PivotConstants {
    // Motor CAN ID
    public static final int MOTOR_ID = 5;

    // Position setpoints (encoder counts)
    public static final int SHOOTING_HIGH_POS = 8000;
    public static final int SHOOTING_LOW_POS = 73000;
    public static final int INTAKE_POS = 160000;

    // Position tolerance
    public static final int POSITION_TOLERANCE = 5000;

    // Motor configuration
    public static final boolean INVERTED = false;
    public static final boolean SENSOR_PHASE = true;
    public static final double PEAK_OUTPUT_FORWARD = 1.0;
    public static final double PEAK_OUTPUT_REVERSE = -1.0;

    // PID constants
    public static final int PID_SLOT = 0;
    public static final int TIMEOUT_MS = 30;
    public static final double MOTOR_F = 0.02325;
    public static final double MOTOR_P = 2.0;
    public static final double MOTOR_I = 0.0008;
    public static final double MOTOR_D = 2.0;

    // MotionMagic constants
    public static final int MOTION_CRUISE_VELOCITY = 100000;
    public static final int MOTION_ACCELERATION = 60000;
    public static final int MOTION_S_CURVE_STRENGTH = 0;
  }

  public static class ShieldConstants {
    // ==========================================
    // CONFIGURATION
    // ==========================================
    // Set to TRUE if the shield motor is installed.
    // Set to FALSE to disable all shield code (saves CAN/CPU usage).
    public static final boolean SHIELD_ENABLED = false;

    // Motor CAN ID
    public static final int MOTOR_ID = 6;

    // Position setpoints (encoder counts)
    public static final double DOWN_POS = 0;
    public static final double MID_POS = 30;
    public static final double UP_POS = 60;

    // Motor configuration
    public static final boolean INVERTED = false;
    public static final boolean SENSOR_PHASE = false;
    public static final double PEAK_OUTPUT_FORWARD = 0.6;
    public static final double PEAK_OUTPUT_REVERSE = -0.6;

    // PID constants
    public static final int PID_SLOT = 0;
    public static final int TIMEOUT_MS = 30;
    public static final double MOTOR_F = 0;
    public static final double MOTOR_P = 80;
    public static final double MOTOR_I = 0;
    public static final double MOTOR_D = 0;

    // Initial position
    public static final double INITIAL_POSITION = 30;
  }

  public static class ShooterConstants {
    // Motor CAN IDs
    public static final int SHOOTER_MOTOR_ID = 8;
    public static final int FEEDER_MOTOR_ID = 7;

    // Motor inversions
    public static final boolean SHOOTER_INVERTED = true;
    public static final boolean FEEDER_INVERTED = false;

    // Speed setpoints
    public static final double SHOOTER_SPEED = 1.0;
    public static final double SHOOTER_REVERSE_SPEED = -0.8;
    public static final double FEEDER_SPEED = 1.0;
    public static final double FEEDER_REVERSE_SPEED = -0.6;
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
    public static final double SHOOTER_SPINUP_TIME = 1.0; // seconds
    public static final double FEED_TIME = 0.5; // seconds
    public static final double WAIT_TIME = 10.0; // seconds for Auto 5/6
  }
}