// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import java.util.function.DoubleSupplier;

/** Mecanum drive subsystem for the 2024 robot. */
public class DriveSubsystem extends SubsystemBase {
  // The "Universal Remote" that controls our motors
  private final DriveIO m_io;

  // Gyroscope for field-centric driving (Knows where "North" is)
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Drive modes
  private boolean m_velocityMode = false;
  private boolean m_fieldCentric = false;

  // Target positions for autonomous
  private double m_leftTargetPosition = 0;
  private double m_rightTargetPosition = 0;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // STARTUP LOGIC:
    // This decides WHICH robot we are running on based on Constants.java
    switch (DriveConstants.CURRENT_DRIVETRAIN_TYPE) {
      case SPARK_MAX_NEO:
        // Load the SparkMax code (Competition Bot)
        m_io = new DriveIOSparkMax();
        break;
      case TALON_SRX_CIM:
      default:
        // Load the TalonSRX code (Practice Bot)
        m_io = new DriveIOTalon();
        break;
    }
  }

  /**
   * Drives the robot using mecanum drive kinematics.
   *
   * @param yAxis Forward/backward axis (positive = forward)
   * @param xAxis Left/right axis (positive = right)
   * @param zAxis Rotation axis (positive = clockwise)
   */
  public void drive(double yAxis, double xAxis, double zAxis) {
    // 1. Apply deadband (Ignore tiny joystick movements)
    yAxis = MathUtil.applyDeadband(yAxis, OperatorConstants.DEADBAND_Y);
    xAxis = MathUtil.applyDeadband(xAxis, OperatorConstants.DEADBAND);
    zAxis = MathUtil.applyDeadband(zAxis, OperatorConstants.DEADBAND);

    // 2. Field Centric Math
    // If on, "Forward" is always away from the driver, regardless of how the robot is facing.
    if (m_fieldCentric) {
      double heading = Math.toRadians(m_gyro.getAngle());
      double cosA = Math.cos(-heading);
      double sinA = Math.sin(-heading);
      double fieldY = yAxis;
      double fieldX = xAxis;
      yAxis = fieldY * cosA - fieldX * sinA;
      xAxis = fieldY * sinA + fieldX * cosA;
    }

    // 3. Mecanum Math (Calculate power for each wheel)
    double frontLeftOutput = yAxis + xAxis + zAxis;
    double frontRightOutput = yAxis - xAxis - zAxis;
    double rearLeftOutput = yAxis - xAxis + zAxis;
    double rearRightOutput = yAxis + xAxis - zAxis;

    // 4. Normalize (Make sure we don't try to send > 100% power)
    double largestOutput = Math.abs(frontLeftOutput);
    if (largestOutput < Math.abs(frontRightOutput)) largestOutput = Math.abs(frontRightOutput);
    if (largestOutput < Math.abs(rearLeftOutput)) largestOutput = Math.abs(rearLeftOutput);
    if (largestOutput < Math.abs(rearRightOutput)) largestOutput = Math.abs(rearRightOutput);
    
    if (largestOutput > 1) {
      frontLeftOutput = frontLeftOutput / largestOutput;
      frontRightOutput = frontRightOutput / largestOutput;
      rearLeftOutput = rearLeftOutput / largestOutput;
      rearRightOutput = rearRightOutput / largestOutput;
    }

    // 5. Send to Motors via the "Universal Remote"
    if (m_velocityMode) {
      // Velocity Mode: "Spin wheels at X RPM"
      m_io.setVelocities(
          DriveConstants.MAX_WHEEL_VELOCITY * frontLeftOutput,
          DriveConstants.MAX_WHEEL_VELOCITY * rearLeftOutput,
          DriveConstants.MAX_WHEEL_VELOCITY * frontRightOutput,
          DriveConstants.MAX_WHEEL_VELOCITY * rearRightOutput);
    } else {
      // Normal Mode: "Give wheels X% voltage"
      m_io.setVoltages(
          frontLeftOutput * 12.0,
          rearLeftOutput * 12.0,
          frontRightOutput * 12.0,
          rearRightOutput * 12.0);
    }
  }

  /** Stops all drive motors. */
  public void stop() {
    m_io.stop();
  }

  /** Resets all drive encoders to zero. */
  public void resetEncoders() {
    m_io.resetEncoders();
    m_leftTargetPosition = 0;
    m_rightTargetPosition = 0;
  }

  /**
   * Drives to a specific absolute position.
   *
   * @param leftTarget Absolute target position for left side (encoder revolutions)
   * @param rightTarget Absolute target position for right side (encoder revolutions)
   */
  public void driveToPosition(double leftTarget, double rightTarget) {
    // SAFETY: If we don't have sensors, we CANNOT do this. Return immediately.
    if (!DriveConstants.HAS_DRIVE_ENCODERS) return; 

    m_leftTargetPosition = leftTarget;
    m_rightTargetPosition = rightTarget;
    m_io.setPositionTargets(leftTarget, rightTarget);
  }

  /** Returns true if the drive is at the target position. */
  public boolean atTargetPosition() {
    // If no sensors, we assume we are "there" instantly so Auto doesn't get stuck waiting.
    if (!DriveConstants.HAS_DRIVE_ENCODERS) return true;

    double[] positions = m_io.getPositions();
    // Check if both sides are close enough to the target
    return Math.abs(positions[0] - m_leftTargetPosition) < DriveConstants.POSITION_TOLERANCE
        && Math.abs(positions[2] - m_rightTargetPosition) < DriveConstants.POSITION_TOLERANCE;
  }

  /** Toggles velocity mode on/off. */
  public void toggleVelocityMode() {
    m_velocityMode = !m_velocityMode;
  }

  /** Returns current velocity mode state. */
  public boolean getVelocityMode() {
    return m_velocityMode;
  }

  /** Toggles field centric mode on/off. */
  public void toggleFieldCentric() {
    m_fieldCentric = !m_fieldCentric;
  }

  /** Returns current field centric mode state. */
  public boolean getFieldCentric() {
    return m_fieldCentric;
  }

  /** Resets the gyro heading to zero. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // ==================== Command Factories ====================
  // These create "Commands" that the buttons on the controller can call.

  public Command driveCommand(
      DoubleSupplier yAxis, DoubleSupplier xAxis, DoubleSupplier zAxis) {
    return run(() -> drive(yAxis.getAsDouble(), xAxis.getAsDouble(), zAxis.getAsDouble()))
        .withName("Drive");
  }

  public Command resetEncodersCommand() {
    return runOnce(() -> resetEncoders()).withName("Reset Encoders");
  }

  public Command driveToPositionCommand(double leftTarget, double rightTarget) {
    // Safety: If no sensors, create a "Dummy" command that finishes instantly.
    if (!DriveConstants.HAS_DRIVE_ENCODERS) {
        return runOnce(() -> {}).withName("Drive to Position (Skipped)");
    }
    // Normal: Start driving, and wait until we get there.
    return Commands.sequence(
            runOnce(() -> driveToPosition(leftTarget, rightTarget)),
            Commands.waitUntil(() -> atTargetPosition()))
        .withName("Drive to Position");
  }

  public Command toggleVelocityModeCommand() {
    return runOnce(() -> toggleVelocityMode()).withName("Toggle Velocity Mode");
  }

  public Command toggleFieldCentricCommand() {
    return runOnce(() -> toggleFieldCentric()).withName("Toggle Field Centric");
  }

  public Command resetGyroCommand() {
    return runOnce(() -> resetGyro()).withName("Reset Gyro");
  }
}
