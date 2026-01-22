package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.DriveConstants;

public class DriveIOTalon implements DriveIO {
  private final WPI_TalonSRX m_frontLeft;
  private final WPI_TalonSRX m_rearLeft;
  private final WPI_TalonSRX m_frontRight;
  private final WPI_TalonSRX m_rearRight;

  public DriveIOTalon() {
    m_frontLeft = new WPI_TalonSRX(DriveConstants.FRONT_LEFT_MOTOR_ID);
    m_rearLeft = new WPI_TalonSRX(DriveConstants.REAR_LEFT_MOTOR_ID);
    m_frontRight = new WPI_TalonSRX(DriveConstants.FRONT_RIGHT_MOTOR_ID);
    m_rearRight = new WPI_TalonSRX(DriveConstants.REAR_RIGHT_MOTOR_ID);

    configureMotor(m_frontLeft, DriveConstants.FRONT_RIGHT_INVERTED);
    configureMotor(m_rearLeft, DriveConstants.REAR_LEFT_INVERTED);
    configureMotor(m_frontRight, DriveConstants.FRONT_RIGHT_INVERTED);
    configureMotor(m_rearRight, DriveConstants.REAR_RIGHT_INVERTED);
  }

  private void configureMotor(WPI_TalonSRX motor, boolean inverted) {
    motor.configFactoryDefault();
    motor.setInverted(inverted);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configOpenloopRamp(DriveConstants.RAMP_RATE);

    // OPTIMIZATION: Slow down all status frames by default to save CAN bandwidth
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10); // Default 10ms (Critical for control)
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 50); // 50ms (20Hz) for Current/Volt monitoring
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);

    if (DriveConstants.HAS_DRIVE_ENCODERS) {
      // If we have encoders, speed up the feedback frame
      motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

      // Configure Quadrature Encoder
      motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
      motor.setSensorPhase(true); 

      // PID Config
      motor.config_kP(0, DriveConstants.MOTOR_P);
      motor.config_kI(0, DriveConstants.MOTOR_I);
      motor.config_kD(0, DriveConstants.MOTOR_D);
      motor.config_kF(0, DriveConstants.MOTOR_FF);
    }
  }

  @Override
  public void setVoltages(double frontLeftVolts, double rearLeftVolts, double frontRightVolts, double rearRightVolts) {
    m_frontLeft.setVoltage(frontLeftVolts);
    m_rearLeft.setVoltage(rearLeftVolts);
    m_frontRight.setVoltage(frontRightVolts);
    m_rearRight.setVoltage(rearRightVolts);
  }

  @Override
  public void setVelocities(double frontLeftVel, double rearLeftVel, double frontRightVel, double rearRightVel) {
    if (DriveConstants.HAS_DRIVE_ENCODERS) {
      double conversion = 4096.0 / 600.0;
      m_frontLeft.set(ControlMode.Velocity, frontLeftVel * conversion);
      m_rearLeft.set(ControlMode.Velocity, rearLeftVel * conversion);
      m_frontRight.set(ControlMode.Velocity, frontRightVel * conversion);
      m_rearRight.set(ControlMode.Velocity, rearRightVel * conversion);
    } else {
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
       double ticks = 4096.0;
       m_frontLeft.set(ControlMode.Position, leftPos * ticks);
       m_rearLeft.set(ControlMode.Position, leftPos * ticks);
       m_frontRight.set(ControlMode.Position, rightPos * ticks);
       m_rearRight.set(ControlMode.Position, rightPos * ticks);
    }
  }

  @Override
  public double[] getPositions() {
    if (DriveConstants.HAS_DRIVE_ENCODERS) {
       double conversion = 1.0 / 4096.0;
       return new double[] {
           m_frontLeft.getSelectedSensorPosition() * conversion,
           m_rearLeft.getSelectedSensorPosition() * conversion,
           m_frontRight.getSelectedSensorPosition() * conversion,
           m_rearRight.getSelectedSensorPosition() * conversion
       };
    }
    return new double[] {0, 0, 0, 0};
  }

  @Override
  public double[] getVelocities() {
     if (DriveConstants.HAS_DRIVE_ENCODERS) {
       double conversion = 600.0 / 4096.0;
       return new double[] {
           m_frontLeft.getSelectedSensorVelocity() * conversion,
           m_rearLeft.getSelectedSensorVelocity() * conversion,
           m_frontRight.getSelectedSensorVelocity() * conversion,
           m_rearRight.getSelectedSensorVelocity() * conversion
       };
    }
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
