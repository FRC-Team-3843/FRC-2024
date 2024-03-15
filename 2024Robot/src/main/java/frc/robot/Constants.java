package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Constants{
    public static final int controller1Channel = 0;
    public static final int controller2Channel = 1;
    public static final double controllerDeadband = 0.12;

    public static final int driveFrontLeftCanID = 1;
    public static final int driveRearLeftCanID = 2;
    public static final int driveFrontRightCanID = 3;
    public static final int driveRearRightCanID = 4;

    public static final int drivePIDSlot = 0;
    public static final double driveP = .0001;
    public static final double driveI = .000001;
    public static final double driveD = .01;
    public static final double driveFF = 0.000156;
    public static final double driveIZ = 0;
    public static final double driveMaxForwardOutput = 1;
    public static final double driveMaxReverseOutput = -1;
    public static final double driveMaxVelocity = 5000; //rpm
    public static final double driveMaxAcceleration = 8000;
    public static final double driveMinVelocity = 0;
    public static final double driveAllowedError = 0;
    public static final double driveRamp = 0.2;

    public static final double driveMaxWheelVelocity = 5000;

    public static final int pivotCanId = 5;
    public static final boolean pivotMotorInvert = false;
    public static final FeedbackDevice pivotSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    public static final boolean pivotSensorInvert = true;
    public static final double pivotP = 2;
    public static final double pivotI = 0.0008;
    public static final double pivotD = 2;
    public static final double pivotFF = 0.02325;
    public static final double pivotMaxForwardOutput = 1;
    public static final double pivotMaxReverseOutput = -1;
    public static final double pivotMaxVelocity = 100000; 
    public static final double pivotMaxAcceleration = 60000;
    public static final int pivotCurve = 0;
    public static final int[] pivotPos = {8000,73000,160000};

    public static final int shieldCanId = 6;
    public static final boolean shieldMotorInvert = false;
    public static final FeedbackDevice shieldSensor = FeedbackDevice.QuadEncoder;
    public static final boolean shieldSensorInvert = false;
    public static final double shieldP = 80;
    public static final double shieldI = 0;
    public static final double shieldD = 0;
    public static final double shieldFF = 0; 
    public static final double shieldMaxForwardOutput = 0.6;
    public static final double shieldMaxReverseOutput = -0.6;
    public static final double[] shieldPos = {0, 30, 60};

    public static final int feederCanID = 7;
    public static final boolean feedMotorInvert = false;

    public static final int shooterCanID = 8;
    public static final boolean shooterMotorInvert = true;

    public static SendableChooser<String> autoChooser;

    public static final String[] autoList = {"Do Nothing", "Auto 1", "Auto 2", "Auto 3", "Auto 4", "Auto 5", "Auto 6"};
}
