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
import frc.robot.Constants.DriveConstants;

/**
 * Hardware Implementation for REV SparkMax Motor Controllers (NEO Motors).
 * 
 * <p>This file contains all the specific code to talk to SparkMaxs.
 * It uses the built-in encoders on the NEO motors for speed and position.
 */
public class DriveIOSparkMax implements DriveIO {
  // The actual motor controller objects
  private final SparkMax m_frontLeft;
  private final SparkMax m_rearLeft;
  private final SparkMax m_frontRight;
  private final SparkMax m_rearRight;

  // PID Controllers (The "Brains" inside the SparkMax that hold speed/position)
  private final SparkClosedLoopController m_frontLeftPID;
  private final SparkClosedLoopController m_rearLeftPID;
  private final SparkClosedLoopController m_frontRightPID;
  private final SparkClosedLoopController m_rearRightPID;

  // The built-in sensors
  private final RelativeEncoder m_frontLeftEncoder;
  private final RelativeEncoder m_rearLeftEncoder;
  private final RelativeEncoder m_frontRightEncoder;
  private final RelativeEncoder m_rearRightEncoder;

  public DriveIOSparkMax() {
    // 1. Initialize motors using IDs from Constants
    m_frontLeft = new SparkMax(DriveConstants.FRONT_LEFT_MOTOR_ID, MotorType.kBrushless);
    m_rearLeft = new SparkMax(DriveConstants.REAR_LEFT_MOTOR_ID, MotorType.kBrushless);
    m_frontRight = new SparkMax(DriveConstants.FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);
    m_rearRight = new SparkMax(DriveConstants.REAR_RIGHT_MOTOR_ID, MotorType.kBrushless);

    // 2. Configure each motor (set PID gains, current limits, etc.)
    configureMotor(m_frontRight, DriveConstants.FRONT_RIGHT_INVERTED);
    m_frontRightPID = m_frontRight.getClosedLoopController();
    m_frontRightEncoder = m_frontRight.getEncoder();

    configureMotor(m_rearRight, DriveConstants.REAR_RIGHT_INVERTED);
    m_rearRightPID = m_rearRight.getClosedLoopController();
    m_rearRightEncoder = m_rearRight.getEncoder();

    configureMotor(m_frontLeft, DriveConstants.FRONT_LEFT_INVERTED);
    m_frontLeftPID = m_frontLeft.getClosedLoopController();
    m_frontLeftEncoder = m_frontLeft.getEncoder();

    configureMotor(m_rearLeft, DriveConstants.REAR_LEFT_INVERTED);
    m_rearLeftPID = m_rearLeft.getClosedLoopController();
    m_rearLeftEncoder = m_rearLeft.getEncoder();
  }

  /**
   * Helper method to apply settings to a SparkMax.
   * This uses the new 2026 "Config" object pattern.
   */
  private void configureMotor(SparkMax motor, boolean inverted) {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(inverted);
    config.openLoopRampRate(DriveConstants.RAMP_RATE); // How fast we accelerate

    // PID Settings (P, I, D) for the internal controller
    config.closedLoop
        .p(DriveConstants.MOTOR_P)
        .i(DriveConstants.MOTOR_I)
        .iZone(DriveConstants.MOTOR_I_ZONE)
        .d(DriveConstants.MOTOR_D)
        .outputRange(DriveConstants.MOTOR_MIN_OUTPUT, DriveConstants.MOTOR_MAX_OUTPUT);

    config.closedLoop.feedForward
        .kV(DriveConstants.MOTOR_FF);

    // Smart Motion (Smooth movement for Auto)
    config.closedLoop.maxMotion
        .cruiseVelocity(DriveConstants.MAX_VELOCITY)
        .maxAcceleration(DriveConstants.MAX_ACCELERATION)
        .allowedProfileError(DriveConstants.ALLOWED_ERROR);

    // Send the config to the motor
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltages(double frontLeftVolts, double rearLeftVolts, double frontRightVolts, double rearRightVolts) {
    // Send voltage directly
    m_frontLeft.setVoltage(frontLeftVolts);
    m_rearLeft.setVoltage(rearLeftVolts);
    m_frontRight.setVoltage(frontRightVolts);
    m_rearRight.setVoltage(rearRightVolts);
  }

  @Override
  public void setVelocities(double frontLeftVel, double rearLeftVel, double frontRightVel, double rearRightVel) {
    // Tell the internal PID controller to target a specific velocity (RPM)
    m_frontLeftPID.setSetpoint(frontLeftVel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    m_rearLeftPID.setSetpoint(rearLeftVel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    m_frontRightPID.setSetpoint(frontRightVel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
    m_rearRightPID.setSetpoint(rearRightVel, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setPositionTargets(double leftPos, double rightPos) {
    // Tell the internal PID controller to target a specific position (Rotations)
    m_frontLeftPID.setSetpoint(leftPos, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    m_rearLeftPID.setSetpoint(leftPos, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    m_frontRightPID.setSetpoint(rightPos, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    m_rearRightPID.setSetpoint(rightPos, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  @Override
  public double[] getPositions() {
    // Read directly from the encoders
    return new double[] {
      m_frontLeftEncoder.getPosition(),
      m_rearLeftEncoder.getPosition(),
      m_frontRightEncoder.getPosition(),
      m_rearRightEncoder.getPosition()
    };
  }

  @Override
  public double[] getVelocities() {
    // Read directly from the encoders
    return new double[] {
      m_frontLeftEncoder.getVelocity(),
      m_rearLeftEncoder.getVelocity(),
      m_frontRightEncoder.getVelocity(),
      m_rearRightEncoder.getVelocity()
    };
  }

  @Override
  public void resetEncoders() {
    m_frontLeftEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
  }

  @Override
  public void stop() {
    m_frontLeft.set(0);
    m_rearLeft.set(0);
    m_frontRight.set(0);
    m_rearRight.set(0);
  }
}