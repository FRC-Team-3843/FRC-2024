// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import java.util.function.DoubleSupplier;

/** Mecanum drive subsystem for the 2024 robot. */
public class DriveSubsystem extends SubsystemBase {
  // Drive motors
  private final SparkMax m_frontLeft;
  private final SparkMax m_rearLeft;
  private final SparkMax m_frontRight;
  private final SparkMax m_rearRight;

  // PID controllers
  private final SparkClosedLoopController m_frontLeftPID;
  private final SparkClosedLoopController m_rearLeftPID;
  private final SparkClosedLoopController m_frontRightPID;
  private final SparkClosedLoopController m_rearRightPID;

  // Encoders
  private final RelativeEncoder m_frontLeftEncoder;
  private final RelativeEncoder m_rearLeftEncoder;
  private final RelativeEncoder m_frontRightEncoder;
  private final RelativeEncoder m_rearRightEncoder;

  // Drive mode
  private boolean m_velocityMode = false;

  // Target positions for autonomous
  private double m_leftTargetPosition = 0;
  private double m_rightTargetPosition = 0;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Initialize motors
    m_frontLeft = new SparkMax(DriveConstants.FRONT_LEFT_MOTOR_ID, MotorType.kBrushless);
    m_rearLeft = new SparkMax(DriveConstants.REAR_LEFT_MOTOR_ID, MotorType.kBrushless);
    m_frontRight = new SparkMax(DriveConstants.FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);
    m_rearRight = new SparkMax(DriveConstants.REAR_RIGHT_MOTOR_ID, MotorType.kBrushless);

    // Configure front right motor
    configureMotor(m_frontRight, DriveConstants.FRONT_RIGHT_INVERTED);
    m_frontRightPID = m_frontRight.getClosedLoopController();
    m_frontRightEncoder = m_frontRight.getEncoder();

    // Configure rear right motor
    configureMotor(m_rearRight, DriveConstants.REAR_RIGHT_INVERTED);
    m_rearRightPID = m_rearRight.getClosedLoopController();
    m_rearRightEncoder = m_rearRight.getEncoder();

    // Configure front left motor
    configureMotor(m_frontLeft, DriveConstants.FRONT_LEFT_INVERTED);
    m_frontLeftPID = m_frontLeft.getClosedLoopController();
    m_frontLeftEncoder = m_frontLeft.getEncoder();

    // Configure rear left motor
    configureMotor(m_rearLeft, DriveConstants.REAR_LEFT_INVERTED);
    m_rearLeftPID = m_rearLeft.getClosedLoopController();
    m_rearLeftEncoder = m_rearLeft.getEncoder();
  }

  /** Configures a motor with all necessary parameters. */
  private void configureMotor(SparkMax motor, boolean inverted) {
    SparkMaxConfig config = new SparkMaxConfig();

    // Motor output configuration
    config.inverted(inverted);
    config.openLoopRampRate(DriveConstants.RAMP_RATE);

    // Closed loop configuration
    config.closedLoop
        .p(DriveConstants.MOTOR_P)
        .i(DriveConstants.MOTOR_I)
        .iZone(DriveConstants.MOTOR_I_ZONE)
        .d(DriveConstants.MOTOR_D)
        .outputRange(DriveConstants.MOTOR_MIN_OUTPUT, DriveConstants.MOTOR_MAX_OUTPUT);

    config.closedLoop.feedForward
        .kV(DriveConstants.MOTOR_FF);

    // Smart Motion configuration
    config.closedLoop.maxMotion
        .cruiseVelocity(DriveConstants.MAX_VELOCITY)
        .maxAcceleration(DriveConstants.MAX_ACCELERATION)
        .allowedProfileError(DriveConstants.ALLOWED_ERROR);

    // Apply configuration
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Drives the robot using mecanum drive kinematics.
   *
   * @param yAxis Forward/backward axis (positive = forward)
   * @param xAxis Left/right axis (positive = right)
   * @param zAxis Rotation axis (positive = clockwise)
   */
  public void drive(double yAxis, double xAxis, double zAxis) {
    // Apply deadband
    yAxis = MathUtil.applyDeadband(yAxis, OperatorConstants.DEADBAND_Y);
    xAxis = MathUtil.applyDeadband(xAxis, OperatorConstants.DEADBAND);
    zAxis = MathUtil.applyDeadband(zAxis, OperatorConstants.DEADBAND);

    // Calculate mecanum drive outputs
    double frontLeftOutput = yAxis + xAxis + zAxis;
    double frontRightOutput = yAxis - xAxis - zAxis;
    double rearLeftOutput = yAxis - xAxis + zAxis;
    double rearRightOutput = yAxis + xAxis - zAxis;

    // Normalize outputs
    double largestOutput = Math.abs(frontLeftOutput);
    if (largestOutput < Math.abs(frontRightOutput)) {
      largestOutput = Math.abs(frontRightOutput);
    }
    if (largestOutput < Math.abs(rearLeftOutput)) {
      largestOutput = Math.abs(rearLeftOutput);
    }
    if (largestOutput < Math.abs(rearRightOutput)) {
      largestOutput = Math.abs(rearRightOutput);
    }
    if (largestOutput > 1) {
      frontLeftOutput = frontLeftOutput / largestOutput;
      frontRightOutput = frontRightOutput / largestOutput;
      rearLeftOutput = rearLeftOutput / largestOutput;
      rearRightOutput = rearRightOutput / largestOutput;
    }

    // Set motor outputs
    if (m_velocityMode) {
      m_frontLeftPID.setSetpoint(
          DriveConstants.MAX_WHEEL_VELOCITY * frontLeftOutput,
          ControlType.kMAXMotionVelocityControl,
          ClosedLoopSlot.kSlot0);
      m_frontRightPID.setSetpoint(
          DriveConstants.MAX_WHEEL_VELOCITY * frontRightOutput,
          ControlType.kMAXMotionVelocityControl,
          ClosedLoopSlot.kSlot0);
      m_rearLeftPID.setSetpoint(
          DriveConstants.MAX_WHEEL_VELOCITY * rearLeftOutput,
          ControlType.kMAXMotionVelocityControl,
          ClosedLoopSlot.kSlot0);
      m_rearRightPID.setSetpoint(
          DriveConstants.MAX_WHEEL_VELOCITY * rearRightOutput,
          ControlType.kMAXMotionVelocityControl,
          ClosedLoopSlot.kSlot0);
    } else {
      m_frontLeft.set(frontLeftOutput);
      m_frontRight.set(frontRightOutput);
      m_rearLeft.set(rearLeftOutput);
      m_rearRight.set(rearRightOutput);
    }
  }

  /** Stops all drive motors. */
  public void stop() {
    m_frontLeft.set(0);
    m_frontRight.set(0);
    m_rearLeft.set(0);
    m_rearRight.set(0);
  }

  /** Resets all drive encoders to zero. */
  public void resetEncoders() {
    m_frontLeftEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
    m_leftTargetPosition = 0;
    m_rightTargetPosition = 0;
  }

  /**
   * Drives to a specific absolute position using SmartMotion.
   *
   * @param leftTarget Absolute target position for left side (encoder revolutions)
   * @param rightTarget Absolute target position for right side (encoder revolutions)
   */
  public void driveToPosition(double leftTarget, double rightTarget) {
    m_leftTargetPosition = leftTarget;
    m_rightTargetPosition = rightTarget;

    m_frontLeftPID.setSetpoint(
        m_leftTargetPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    m_rearLeftPID.setSetpoint(
        m_leftTargetPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    m_frontRightPID.setSetpoint(
        m_rightTargetPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    m_rearRightPID.setSetpoint(
        m_rightTargetPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  /** Returns true if the drive is at the target position. */
  public boolean atTargetPosition() {
    return Math.abs(m_frontLeftEncoder.getPosition() - m_leftTargetPosition)
            < DriveConstants.POSITION_TOLERANCE
        && Math.abs(m_frontRightEncoder.getPosition() - m_rightTargetPosition)
            < DriveConstants.POSITION_TOLERANCE;
  }

  /** Toggles velocity mode on/off. */
  public void toggleVelocityMode() {
    m_velocityMode = !m_velocityMode;
  }

  /** Returns current velocity mode state. */
  public boolean getVelocityMode() {
    return m_velocityMode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // ==================== Command Factories ====================

  /**
   * Creates a command to drive the robot with joystick inputs.
   *
   * @param yAxis Supplier for forward/backward axis
   * @param xAxis Supplier for left/right axis
   * @param zAxis Supplier for rotation axis
   * @return Command to drive the robot
   */
  public Command driveCommand(
      DoubleSupplier yAxis, DoubleSupplier xAxis, DoubleSupplier zAxis) {
    return run(() -> drive(yAxis.getAsDouble(), xAxis.getAsDouble(), zAxis.getAsDouble()))
        .withName("Drive");
  }

  /** Returns a command to reset the drive encoders. */
  public Command resetEncodersCommand() {
    return runOnce(() -> resetEncoders()).withName("Reset Encoders");
  }

  /**
   * Returns a command to drive to a specific absolute position.
   *
   * @param leftTarget Absolute target position for left side
   * @param rightTarget Absolute target position for right side
   * @return Command that drives to position and waits until arrival
   */
  public Command driveToPositionCommand(double leftTarget, double rightTarget) {
    return Commands.sequence(
            runOnce(() -> driveToPosition(leftTarget, rightTarget)),
            Commands.waitUntil(() -> atTargetPosition()))
        .withName("Drive to Position");
  }

  /** Returns a command to toggle velocity mode. */
  public Command toggleVelocityModeCommand() {
    return runOnce(() -> toggleVelocityMode()).withName("Toggle Velocity Mode");
  }
}
