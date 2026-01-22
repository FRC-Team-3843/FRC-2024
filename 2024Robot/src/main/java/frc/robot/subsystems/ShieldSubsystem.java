// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShieldConstants;

/** Shield subsystem for controlling the intake shield mechanism. */
public class ShieldSubsystem extends SubsystemBase {
  // If SHIELD_ENABLED is false, this remains null to save resources.
  private final TalonSRX m_shieldMotor;

  /** Creates a new ShieldSubsystem. */
  public ShieldSubsystem() {
    if (!ShieldConstants.SHIELD_ENABLED) {
      m_shieldMotor = null;
      System.out.println("WARNING: Shield Subsystem is DISABLED in Constants.java");
      return;
    }

    m_shieldMotor = new TalonSRX(ShieldConstants.MOTOR_ID);

    // Factory default first
    m_shieldMotor.configFactoryDefault();

    // OPTIMIZATION: Slow down CAN status frames for data we don't need frequently
    m_shieldMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_shieldMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    m_shieldMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100);

    // Configure sensor feedback device
    m_shieldMotor.configSelectedFeedbackSensor(
        FeedbackDevice.CTRE_MagEncoder_Relative,
        ShieldConstants.PID_SLOT,
        ShieldConstants.TIMEOUT_MS);

    // Configure sensor and motor inversion
    m_shieldMotor.setSensorPhase(ShieldConstants.SENSOR_PHASE);
    m_shieldMotor.setInverted(ShieldConstants.INVERTED);

    // Configure peak outputs
    m_shieldMotor.configPeakOutputForward(ShieldConstants.PEAK_OUTPUT_FORWARD);
    m_shieldMotor.configPeakOutputReverse(ShieldConstants.PEAK_OUTPUT_REVERSE);

    // Configure PID
    m_shieldMotor.config_kF(
        ShieldConstants.PID_SLOT, ShieldConstants.MOTOR_F, ShieldConstants.TIMEOUT_MS);
    m_shieldMotor.config_kP(
        ShieldConstants.PID_SLOT, ShieldConstants.MOTOR_P, ShieldConstants.TIMEOUT_MS);
    m_shieldMotor.config_kI(
        ShieldConstants.PID_SLOT, ShieldConstants.MOTOR_I, ShieldConstants.TIMEOUT_MS);
    m_shieldMotor.config_kD(
        ShieldConstants.PID_SLOT, ShieldConstants.MOTOR_D, ShieldConstants.TIMEOUT_MS);

    // Set initial position
    m_shieldMotor.setSelectedSensorPosition(
        ShieldConstants.INITIAL_POSITION, ShieldConstants.PID_SLOT, ShieldConstants.TIMEOUT_MS);
  }

  /**
   * Sets the shield position.
   *
   * @param position Target position
   */
  public void setPosition(double position) {
    if (!ShieldConstants.SHIELD_ENABLED) return;
    m_shieldMotor.set(ControlMode.Position, position);
  }

  /** Moves shield to down position. */
  public void moveDown() {
    setPosition(ShieldConstants.DOWN_POS);
  }

  /** Moves shield to mid position. */
  public void moveMid() {
    setPosition(ShieldConstants.MID_POS);
  }

  /** Moves shield to up position. */
  public void moveUp() {
    setPosition(ShieldConstants.UP_POS);
  }

  /** Returns the current shield position. */
  public double getPosition() {
    if (!ShieldConstants.SHIELD_ENABLED) return 0.0;
    return m_shieldMotor.getSelectedSensorPosition();
  }

  /** Returns true if the shield is at the down position. */
  public boolean isAtDown() {
    if (!ShieldConstants.SHIELD_ENABLED) return true; // Assume success to not block auto
    return Math.abs(getPosition() - ShieldConstants.DOWN_POS) < 1.0;
  }

  /** Returns true if the shield is at the mid position. */
  public boolean isAtMid() {
    if (!ShieldConstants.SHIELD_ENABLED) return true;
    return Math.abs(getPosition() - ShieldConstants.MID_POS) < 1.0;
  }

  /** Returns true if the shield is at the up position. */
  public boolean isAtUp() {
    if (!ShieldConstants.SHIELD_ENABLED) return true;
    return Math.abs(getPosition() - ShieldConstants.UP_POS) < 1.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // ==================== Command Factories ====================
  // If disabled, these return "InstantCommand" that does nothing (runOnce -> {}).
  // This effectively removes them from any parallel groups they are in.

  /** Returns a command to move the shield to the down position. */
  public Command moveDownCommand() {
    if (!ShieldConstants.SHIELD_ENABLED) return runOnce(() -> {}).withName("Shield Disabled");
    return runOnce(() -> moveDown()).withName("Shield Down");
  }

  /** Returns a command to move the shield to the mid position. */
  public Command moveMidCommand() {
    if (!ShieldConstants.SHIELD_ENABLED) return runOnce(() -> {}).withName("Shield Disabled");
    return runOnce(() -> moveMid()).withName("Shield Mid");
  }

  /** Returns a command to move the shield to the up position. */
  public Command moveUpCommand() {
    if (!ShieldConstants.SHIELD_ENABLED) return runOnce(() -> {}).withName("Shield Disabled");
    return runOnce(() -> moveUp()).withName("Shield Up");
  }

  /** Returns a command that waits until the shield reaches the down position. */
  public Command waitUntilAtDown() {
    if (!ShieldConstants.SHIELD_ENABLED) return runOnce(() -> {}).withName("Shield Disabled");
    return run(() -> {}).until(() -> isAtDown()).withName("Wait Until Shield Down");
  }

  /** Returns a command that waits until the shield reaches the mid position. */
  public Command waitUntilAtMid() {
    if (!ShieldConstants.SHIELD_ENABLED) return runOnce(() -> {}).withName("Shield Disabled");
    return run(() -> {}).until(() -> isAtMid()).withName("Wait Until Shield Mid");
  }

  /** Returns a command that waits until the shield reaches the up position. */
  public Command waitUntilAtUp() {
    if (!ShieldConstants.SHIELD_ENABLED) return runOnce(() -> {}).withName("Shield Disabled");
    return run(() -> {}).until(() -> isAtUp()).withName("Wait Until Shield Up");
  }
}
