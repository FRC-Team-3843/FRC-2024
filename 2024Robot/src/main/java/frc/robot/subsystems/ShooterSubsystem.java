// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/** Shooter subsystem for controlling the shooter and feeder motors. */
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_shooterMotor;
  private final TalonSRX m_feederMotor;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    if (!ShooterConstants.SHOOTER_ENABLED) {
        m_shooterMotor = null;
        m_feederMotor = null;
        System.out.println("WARNING: Shooter Subsystem is DISABLED in Constants.java");
        return;
    }

    m_shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID);
    m_feederMotor = new TalonSRX(ShooterConstants.FEEDER_MOTOR_ID);

    // Configure shooter motor (TalonFX/Phoenix6)
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted =
        ShooterConstants.SHOOTER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    m_shooterMotor.getConfigurator().apply(shooterConfig);

    // OPTIMIZATION: Automatically throttle down CAN signals we aren't using (feedback, etc.)
    m_shooterMotor.optimizeBusUtilization();

    // Configure feeder motor (TalonSRX/Phoenix5)
    m_feederMotor.configFactoryDefault();
    m_feederMotor.setInverted(ShooterConstants.FEEDER_INVERTED);

    // OPTIMIZATION: Slow down all status frames for the feeder since we don't need feedback
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    m_feederMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
  }

  /**
   * Spins the shooter at the specified speed.
   *
   * @param speed Speed to spin (-1.0 to 1.0)
   */
  public void spinShooter(double speed) {
    if (!ShooterConstants.SHOOTER_ENABLED) return;
    m_shooterMotor.set(speed);
  }

  /** Spins the shooter at full forward speed. */
  public void spinUp() {
    spinShooter(ShooterConstants.SHOOTER_SPEED);
  }

  /** Spins the shooter in reverse. */
  public void spinReverse() {
    spinShooter(ShooterConstants.SHOOTER_REVERSE_SPEED);
  }

  /** Stops the shooter motor. */
  public void stopShooter() {
    if (!ShooterConstants.SHOOTER_ENABLED) return;
    m_shooterMotor.set(0);
  }

  /**
   * Runs the feeder at the specified speed.
   *
   * @param speed Speed to run (-1.0 to 1.0)
   */
  public void runFeeder(double speed) {
    if (!ShooterConstants.SHOOTER_ENABLED) return;
    m_feederMotor.set(ControlMode.PercentOutput, speed);
  }

  /** Runs the feeder at full forward speed. */
  public void feed() {
    runFeeder(ShooterConstants.FEEDER_SPEED);
  }

  /** Runs the feeder in reverse. */
  public void feedReverse() {
    runFeeder(ShooterConstants.FEEDER_REVERSE_SPEED);
  }

  /** Stops the feeder motor. */
  public void stopFeeder() {
    if (!ShooterConstants.SHOOTER_ENABLED) return;
    m_feederMotor.set(ControlMode.PercentOutput, 0);
  }

  /** Stops both shooter and feeder motors. */
  public void stopAll() {
    stopShooter();
    stopFeeder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // ==================== Command Factories ====================

  /** Returns a command to spin up the shooter. */
  public Command spinUpCommand() {
    if (!ShooterConstants.SHOOTER_ENABLED) return runOnce(() -> {}).withName("Shooter Disabled");
    return runOnce(() -> spinUp()).withName("Spin Up Shooter");
  }

  /** Returns a command to spin the shooter in reverse. */
  public Command spinReverseCommand() {
    if (!ShooterConstants.SHOOTER_ENABLED) return runOnce(() -> {}).withName("Shooter Disabled");
    return runOnce(() -> spinReverse()).withName("Spin Shooter Reverse");
  }

  /** Returns a command to stop the shooter. */
  public Command stopShooterCommand() {
    if (!ShooterConstants.SHOOTER_ENABLED) return runOnce(() -> {}).withName("Shooter Disabled");
    return runOnce(() -> stopShooter()).withName("Stop Shooter");
  }

  /** Returns a command to run the feeder. */
  public Command feedCommand() {
    if (!ShooterConstants.SHOOTER_ENABLED) return runOnce(() -> {}).withName("Shooter Disabled");
    return runOnce(() -> feed()).withName("Feed");
  }

  /** Returns a command to run the feeder in reverse. */
  public Command feedReverseCommand() {
    if (!ShooterConstants.SHOOTER_ENABLED) return runOnce(() -> {}).withName("Shooter Disabled");
    return runOnce(() -> feedReverse()).withName("Feed Reverse");
  }

  /** Returns a command to stop the feeder. */
  public Command stopFeederCommand() {
    if (!ShooterConstants.SHOOTER_ENABLED) return runOnce(() -> {}).withName("Shooter Disabled");
    return runOnce(() -> stopFeeder()).withName("Stop Feeder");
  }

  /** Returns a command to stop both shooter and feeder. */
  public Command stopCommand() {
    if (!ShooterConstants.SHOOTER_ENABLED) return runOnce(() -> {}).withName("Shooter Disabled");
    return runOnce(() -> stopAll()).withName("Stop All");
  }
}
