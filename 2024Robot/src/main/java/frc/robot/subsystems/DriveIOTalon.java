package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.DriveConstants;

/**
 * Hardware Implementation for CTRE TalonSRX Motor Controllers (CIM Motors).
 * 
 * <p>This file handles the Practice Bot.
 * SPECIAL FEATURE: It checks DriveConstants.HAS_DRIVE_ENCODERS.
 * - If TRUE: It behaves like a normal closed-loop system.
 * - If FALSE: It "Simulates" velocity by just setting voltage, and reports 0 for position.
 *   This prevents the robot from crashing when we run it without sensors.
 */
public class DriveIOTalon implements DriveIO {
  private final WPI_TalonSRX m_frontLeft;
  private final WPI_TalonSRX m_rearLeft;
  private final WPI_TalonSRX m_frontRight;
  private final WPI_TalonSRX m_rearRight;

  public DriveIOTalon() {
    // 1. Initialize motors
    m_frontLeft = new WPI_TalonSRX(DriveConstants.FRONT_LEFT_MOTOR_ID);
    m_rearLeft = new WPI_TalonSRX(DriveConstants.REAR_LEFT_MOTOR_ID);
    m_frontRight = new WPI_TalonSRX(DriveConstants.FRONT_RIGHT_MOTOR_ID);
    m_rearRight = new WPI_TalonSRX(DriveConstants.REAR_RIGHT_MOTOR_ID);

    // 2. Configure motors
    configureMotor(m_frontLeft, DriveConstants.FRONT_LEFT_INVERTED);
    configureMotor(m_rearLeft, DriveConstants.REAR_LEFT_INVERTED);
    configureMotor(m_frontRight, DriveConstants.FRONT_RIGHT_INVERTED);
    configureMotor(m_rearRight, DriveConstants.REAR_RIGHT_INVERTED);
  }

  private void configureMotor(WPI_TalonSRX motor, boolean inverted) {
    motor.configFactoryDefault(); // Reset to clean state
    motor.setInverted(inverted);
    motor.setNeutralMode(NeutralMode.Brake); // Brake when 0 power (stops quickly)
    motor.configOpenloopRamp(DriveConstants.RAMP_RATE); // Ramp up speed slowly to prevent wheel slip

    // Only set up sensors if we actually have them!
    if (DriveConstants.HAS_DRIVE_ENCODERS) {
      // Configure Quadrature Encoder (Plugged into data port)
      motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
      motor.setSensorPhase(true); 

      // PID Config (P, I, D) for the Talon's internal computer
      motor.config_kP(0, DriveConstants.MOTOR_P);
      motor.config_kI(0, DriveConstants.MOTOR_I);
      motor.config_kD(0, DriveConstants.MOTOR_D);
      motor.config_kF(0, DriveConstants.MOTOR_FF);
    }
  }

  @Override
  public void setVoltages(double frontLeftVolts, double rearLeftVolts, double frontRightVolts, double rearRightVolts) {
    // Apply voltage directly
    m_frontLeft.setVoltage(frontLeftVolts);
    m_rearLeft.setVoltage(rearLeftVolts);
    m_frontRight.setVoltage(frontRightVolts);
    m_rearRight.setVoltage(rearRightVolts);
  }

  @Override
  public void setVelocities(double frontLeftVel, double rearLeftVel, double frontRightVel, double rearRightVel) {
    if (DriveConstants.HAS_DRIVE_ENCODERS) {
      // REAL MODE: We have sensors, so use the Talon's PID
      // Convert RPM to Talon Native Units (Ticks per 100ms)
      // 1 RPM = 4096 ticks / 60 sec = 68.26 ticks/sec = 6.826 ticks/100ms
      double conversion = 4096.0 / 600.0;
      
      m_frontLeft.set(ControlMode.Velocity, frontLeftVel * conversion);
      m_rearLeft.set(ControlMode.Velocity, rearLeftVel * conversion);
      m_frontRight.set(ControlMode.Velocity, frontRightVel * conversion);
      m_rearRight.set(ControlMode.Velocity, rearRightVel * conversion);
    } else {
      // FAKE MODE: No sensors, so we guess!
      // Math: If Max Velocity is 5000 RPM, and we want 2500 RPM, that is 50% power.
      // 50% of 12 Volts = 6 Volts.
      double maxRPM = DriveConstants.MAX_WHEEL_VELOCITY;
      
      setVoltages(
          (frontLeftVel / maxRPM) * 12.0,
          (rearLeftVel / maxRPM) * 12.0,
          (frontRightVel / maxRPM) * 12.0,
          (rearRightVel / maxRPM) * 12.0
      );
    }
  }

  @Override
  public void setPositionTargets(double leftPos, double rightPos) {
    if (DriveConstants.HAS_DRIVE_ENCODERS) {
       // REAL MODE: Use sensors to go to position
       // Rotations to Ticks (4096 ticks per rotation)
       double ticks = 4096.0;
       m_frontLeft.set(ControlMode.Position, leftPos * ticks);
       m_rearLeft.set(ControlMode.Position, leftPos * ticks);
       m_frontRight.set(ControlMode.Position, rightPos * ticks);
       m_rearRight.set(ControlMode.Position, rightPos * ticks);
    }
    // FAKE MODE: Do nothing. We can't go to a position if we don't know where we are.
  }

  @Override
  public double[] getPositions() {
    if (DriveConstants.HAS_DRIVE_ENCODERS) {
       // Return real positions (Ticks -> Rotations)
       double conversion = 1.0 / 4096.0;
       return new double[] {
           m_frontLeft.getSelectedSensorPosition() * conversion,
           m_rearLeft.getSelectedSensorPosition() * conversion,
           m_frontRight.getSelectedSensorPosition() * conversion,
           m_rearRight.getSelectedSensorPosition() * conversion
       };
    }
    // No sensors? Return 0 so the code doesn't break.
    return new double[] {0, 0, 0, 0};
  }

  @Override
  public double[] getVelocities() {
     if (DriveConstants.HAS_DRIVE_ENCODERS) {
       // Return real speeds (Ticks/100ms -> RPM)
       double conversion = 600.0 / 4096.0;
       return new double[] {
           m_frontLeft.getSelectedSensorVelocity() * conversion,
           m_rearLeft.getSelectedSensorVelocity() * conversion,
           m_frontRight.getSelectedSensorVelocity() * conversion,
           m_rearRight.getSelectedSensorVelocity() * conversion
       };
    }
    // No sensors? Return 0.
    return new double[] {0, 0, 0, 0};
  }

  @Override
  public void resetEncoders() {
    if (DriveConstants.HAS_DRIVE_ENCODERS) {
      m_frontLeft.setSelectedSensorPosition(0);
      m_rearLeft.setSelectedSensorPosition(0);
      m_frontRight.setSelectedSensorPosition(0);
      m_rearRight.setSelectedSensorPosition(0);
    }
  }

  @Override
  public void stop() {
    m_frontLeft.stopMotor();
    m_rearLeft.stopMotor();
    m_frontRight.stopMotor();
    m_rearRight.stopMotor();
  }
}