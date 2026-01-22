// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

/** Pivot subsystem for controlling the shooter pivot mechanism. */
public class PivotSubsystem extends SubsystemBase {
  private final TalonSRX m_pivotMotor;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    m_pivotMotor = new TalonSRX(PivotConstants.MOTOR_ID);

    // Configure sensor feedback device
    m_pivotMotor.configSelectedFeedbackSensor(
        FeedbackDevice.CTRE_MagEncoder_Relative,
        PivotConstants.PID_SLOT,
        PivotConstants.TIMEOUT_MS);

    // Configure sensor and motor inversion
    m_pivotMotor.setSensorPhase(PivotConstants.SENSOR_PHASE);
    m_pivotMotor.setInverted(PivotConstants.INVERTED);

    // Configure peak outputs
    m_pivotMotor.configPeakOutputForward(PivotConstants.PEAK_OUTPUT_FORWARD);
    m_pivotMotor.configPeakOutputReverse(PivotConstants.PEAK_OUTPUT_REVERSE);

    // Configure PID
    m_pivotMotor.config_kF(
        PivotConstants.PID_SLOT, PivotConstants.MOTOR_F, PivotConstants.TIMEOUT_MS);
    m_pivotMotor.config_kP(
        PivotConstants.PID_SLOT, PivotConstants.MOTOR_P, PivotConstants.TIMEOUT_MS);
    m_pivotMotor.config_kI(
        PivotConstants.PID_SLOT, PivotConstants.MOTOR_I, PivotConstants.TIMEOUT_MS);
    m_pivotMotor.config_kD(
        PivotConstants.PID_SLOT, PivotConstants.MOTOR_D, PivotConstants.TIMEOUT_MS);

    // Configure MotionMagic
    m_pivotMotor.configMotionCruiseVelocity(
        PivotConstants.MOTION_CRUISE_VELOCITY, PivotConstants.PID_SLOT);
    m_pivotMotor.configMotionAcceleration(
        PivotConstants.MOTION_ACCELERATION, PivotConstants.TIMEOUT_MS);
    m_pivotMotor.configMotionSCurveStrength(PivotConstants.MOTION_S_CURVE_STRENGTH);

    // Zero the sensor
    m_pivotMotor.setSelectedSensorPosition(0, PivotConstants.PID_SLOT, PivotConstants.TIMEOUT_MS);
  }

  /**
   * Sets the pivot position using MotionMagic.
   *
   * @param position Target position in encoder counts
   */
  public void setPosition(double position) {
    m_pivotMotor.set(ControlMode.MotionMagic, position);
  }

  /** Moves pivot to shooting high position. */
  public void moveToHigh() {
    setPosition(PivotConstants.SHOOTING_HIGH_POS);
  }

  /** Moves pivot to shooting low position. */
  public void moveToLow() {
    setPosition(PivotConstants.SHOOTING_LOW_POS);
  }

  /** Moves pivot to intake position. */
  public void moveToIntake() {
    setPosition(PivotConstants.INTAKE_POS);
  }

  /** Stops the pivot motor. */
  public void stop() {
    m_pivotMotor.set(ControlMode.PercentOutput, 0);
  }

  /** Returns the current pivot position in encoder counts. */
  public double getPosition() {
    return m_pivotMotor.getSelectedSensorPosition();
  }

  /** Returns true if the pivot is at the shooting high position. */
  public boolean isAtShootingHigh() {
    return Math.abs(getPosition() - PivotConstants.SHOOTING_HIGH_POS)
        < PivotConstants.POSITION_TOLERANCE;
  }

  /** Returns true if the pivot is at the shooting low position. */
  public boolean isAtShootingLow() {
    return Math.abs(getPosition() - PivotConstants.SHOOTING_LOW_POS)
        < PivotConstants.POSITION_TOLERANCE;
  }

  /** Returns true if the pivot is at the intake position. */
  public boolean isAtIntake() {
    return Math.abs(getPosition() - PivotConstants.INTAKE_POS)
        < PivotConstants.POSITION_TOLERANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // ==================== Command Factories ====================

  /** Returns a command to move the pivot to the shooting high position. */
  public Command moveToHighCommand() {
    return runOnce(() -> moveToHigh()).withName("Pivot to High");
  }

  /** Returns a command to move the pivot to the shooting low position. */
  public Command moveToLowCommand() {
    return runOnce(() -> moveToLow()).withName("Pivot to Low");
  }

  /** Returns a command to move the pivot to the intake position. */
  public Command moveToIntakeCommand() {
    return runOnce(() -> moveToIntake()).withName("Pivot to Intake");
  }

  /** Returns a command that waits until the pivot reaches the shooting high position. */
  public Command waitUntilAtHigh() {
    return run(() -> {}).until(() -> isAtShootingHigh()).withName("Wait Until at High");
  }

  /** Returns a command that waits until the pivot reaches the shooting low position. */
  public Command waitUntilAtLow() {
    return run(() -> {}).until(() -> isAtShootingLow()).withName("Wait Until at Low");
  }

  /** Returns a command that waits until the pivot reaches the intake position. */
  public Command waitUntilAtIntake() {
    return run(() -> {}).until(() -> isAtIntake()).withName("Wait Until at Intake");
  }
}
