// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Servo;
//import edu.wpi.first.util.sendable.SendableRegistry;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.sendable.SendableRegistry;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {
  

  private MecanumDrive robotDrive;

  private XboxController controller1;

  private final Timer timer = new Timer();

  private CANSparkMax frontLeft, rearLeft, frontRight, rearRight;
  private SparkPIDController frontLeftPID, rearLeftPID, frontRightPID, rearRightPID;
  private RelativeEncoder frontLeftEncoder, rearLeftEncoder, frontRightEncoder, rearRightEncoder;
  private TalonSRX pivotMotor, feederMotor, shooterMotor;

  private Servo hoodServo;

  private double xAxis = 0, yAxis = 0, zAxis = 0;

  private String autoSelection;

  DoublePublisher xPub, yPub, zPub, testDoublePub;
  IntegerPublisher testIntPub;
  StringPublisher autoPub;

  int testInt = 0;
  double testDouble = 0;

  private static final int controllerChannel = 0;

  private static final int pivotMotorIdx = 0;
  private static final int pivotMotorTimeout = 30;

  private static final int shootingHighPosition = 8000;
  private static final int shootingLowPosition = 80000;
  private static final int intakePosition = 165000;

  private static final double servoUp = .4;
  private static final double servoDown = .55;
  private static final double servoIntake = .7;

  private static final double deadband = 0.12;

  private static final int PIDSlot = 0;
  private static final double driveMotorP = 0.0001;
  private static final double driveMotorI = 0;
  private static final double driveMotorKIz = 0;
  private static final double driveMotorD = 0;
  private static final double driveMotorFF = 0.000167;
  private static final double driveMotorMaxOutput = 1;
  private static final double driveMotorMaxVelocity = 3000; //rpm
  private static final double driveMotorMaxAcceleration = 1500;
  private static final double driveMotorMinVelocity = 0;
  private static final double driveMotorAllowedError = 0;

  public static SendableChooser<String> autoChooser;

  private static final String[] autoList = {"Do Nothing", "Auto 1", "Auto 2", "Auto 3"};





  @Override
  public void robotInit() {
    //Initialize Motors
    frontLeft = new CANSparkMax(1,MotorType.kBrushless);
    rearLeft = new CANSparkMax(2,MotorType.kBrushless);
    frontRight = new CANSparkMax(3,MotorType.kBrushless);
    rearRight = new CANSparkMax(4,MotorType.kBrushless);
    
    pivotMotor = new TalonSRX(5);
    shooterMotor = new TalonSRX(6);
    feederMotor = new TalonSRX(7);



    //Setup Pivot Motor
    //Set Sensor Feedback Device
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,pivotMotorIdx,pivotMotorTimeout);
    //Invert Sensor
    pivotMotor.setSensorPhase(true);
    //Invert Motor
    pivotMotor.setInverted(false);
    //Set Maximum Output
    pivotMotor.configPeakOutputForward(1);
    pivotMotor.configPeakOutputReverse(-1);
    //Set PID 
    pivotMotor.config_kF(pivotMotorIdx, 0.02325, pivotMotorTimeout);
		pivotMotor.config_kP(pivotMotorIdx, 2, pivotMotorTimeout);
		pivotMotor.config_kI(pivotMotorIdx, 0.0008, pivotMotorTimeout);
		pivotMotor.config_kD(pivotMotorIdx, 15, pivotMotorTimeout);
    //Set acceleration and vcruise velocity
		pivotMotor.configMotionCruiseVelocity(25000, pivotMotorIdx);
		pivotMotor.configMotionAcceleration(25000, pivotMotorTimeout);
    //Set Smoothing
    pivotMotor.configMotionSCurveStrength(2);
    // Zero the sensor once on robot boot up
		pivotMotor.setSelectedSensorPosition(0, pivotMotorIdx, pivotMotorTimeout);

    // Setup drive motors
    frontRight.setInverted(false);
    frontRightPID = frontRight.getPIDController();
    frontRightEncoder = frontRight.getEncoder();
    frontRightPID.setP(driveMotorP);
    frontRightPID.setI(driveMotorI);
    frontRightPID.setIZone(driveMotorKIz);
    frontRightPID.setD(driveMotorD);
    frontRightPID.setFF(driveMotorFF);
    frontRightPID.setOutputRange(-driveMotorMaxOutput, driveMotorMaxOutput);
    frontRightPID.setSmartMotionMaxVelocity(driveMotorMaxVelocity, PIDSlot);
    frontRightPID.setSmartMotionMaxAccel(driveMotorMaxAcceleration, PIDSlot);
    frontRightPID.setSmartMotionMinOutputVelocity(driveMotorAllowedError, PIDSlot);
    frontRightPID.setSmartMotionMinOutputVelocity(driveMotorMinVelocity, PIDSlot);
    
    rearRight.setInverted(false);
    rearRightPID = rearRight.getPIDController();
    rearRightEncoder = rearRight.getEncoder();
    rearRightPID.setP(driveMotorP);
    rearRightPID.setI(driveMotorI);
    rearRightPID.setIZone(driveMotorKIz);
    rearRightPID.setD(driveMotorD);
    rearRightPID.setFF(driveMotorFF);
    rearRightPID.setOutputRange(-driveMotorMaxOutput, driveMotorMaxOutput);
    rearRightPID.setSmartMotionMaxVelocity(driveMotorMaxVelocity, PIDSlot);
    rearRightPID.setSmartMotionMaxAccel(driveMotorMaxAcceleration, PIDSlot);
    rearRightPID.setSmartMotionMinOutputVelocity(driveMotorAllowedError, PIDSlot);
    rearRightPID.setSmartMotionMinOutputVelocity(driveMotorMinVelocity, PIDSlot);

    frontLeft.setInverted(true);
    frontLeftPID = frontLeft.getPIDController();
    frontLeftEncoder = frontLeft.getEncoder();
    frontLeftPID.setP(driveMotorP);
    frontLeftPID.setI(driveMotorI);
    frontLeftPID.setIZone(driveMotorKIz);
    frontLeftPID.setD(driveMotorD);
    frontLeftPID.setFF(driveMotorFF);
    frontLeftPID.setOutputRange(-driveMotorMaxOutput, driveMotorMaxOutput);
    frontLeftPID.setSmartMotionMaxVelocity(driveMotorMaxVelocity, PIDSlot);
    frontLeftPID.setSmartMotionMaxAccel(driveMotorMaxAcceleration, PIDSlot);
    frontLeftPID.setSmartMotionMinOutputVelocity(driveMotorAllowedError, PIDSlot);
    frontLeftPID.setSmartMotionMinOutputVelocity(driveMotorMinVelocity, PIDSlot);

    rearLeft.setInverted(true);
    rearLeftPID = rearLeft.getPIDController();
    rearLeftEncoder = rearLeft.getEncoder();
    rearLeftPID.setP(driveMotorP);
    rearLeftPID.setI(driveMotorI);
    rearLeftPID.setIZone(driveMotorKIz);
    rearLeftPID.setD(driveMotorD);
    rearLeftPID.setFF(driveMotorFF);
    rearLeftPID.setOutputRange(-driveMotorMaxOutput, driveMotorMaxOutput);
    rearLeftPID.setSmartMotionMaxVelocity(driveMotorMaxVelocity, PIDSlot);
    rearLeftPID.setSmartMotionMaxAccel(driveMotorMaxAcceleration, PIDSlot);
    rearLeftPID.setSmartMotionMinOutputVelocity(driveMotorAllowedError, PIDSlot);
    rearLeftPID.setSmartMotionMinOutputVelocity(driveMotorMinVelocity, PIDSlot);

    //Setup feeder and shoorter motors
    feederMotor.setInverted(false);
    shooterMotor.setInverted(true);

    //Setup hood servo motor
    hoodServo = new Servo(0);

    robotDrive = new MecanumDrive(frontLeft::set, rearLeft::set, frontRight::set, rearRight::set);

    controller1 = new XboxController(controllerChannel);

    //Add auto options to dashboard
    SmartDashboard.putStringArray("Auto List", autoList);  

    // Get the default instance of NetworkTables that was created automatically
    // when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // Get the table within that instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called datatable.
    NetworkTable table = inst.getTable("datatable");

    // Start publishing topics within that table that correspond to the X and Y values
    // for some operation in your program.
    // The topic names are actually "/datatable/x" and "/datatable/y".
    xPub = table.getDoubleTopic("Joystick X Axis").publish();
    yPub = table.getDoubleTopic("Joystick Y Axis").publish();
    zPub = table.getDoubleTopic("Joystick Z Axis").publish();
    testDoublePub = table.getDoubleTopic("Test Double").publish();
    testIntPub = table.getIntegerTopic("Test Int").publish();
    autoPub = table.getStringTopic("Auto Selection").publish();

    SendableRegistry.addChild(robotDrive, frontLeft);
    SendableRegistry.addChild(robotDrive, rearLeft);
    SendableRegistry.addChild(robotDrive, frontRight);
    SendableRegistry.addChild(robotDrive, rearRight);

  }

  @Override
  public void robotPeriodic(){
    // Use the joystick Y axis for forward movement, X axis for lateral
    // movement, and Z axis for rotation.
    yAxis = controller1.getLeftY();
    xAxis = controller1.getRightX();
    zAxis = controller1.getLeftX();
    if (yAxis < deadband && yAxis > -deadband) 
      yAxis = 0;
    if (xAxis < deadband && xAxis > -deadband) 
      xAxis = 0;
    if (zAxis < deadband && zAxis > -deadband) 
      zAxis = 0;
    zAxis = -zAxis;
    xAxis = -xAxis;

    xPub.set(xAxis);
    yPub.set(yAxis);
    zPub.set(zAxis);
    testIntPub.set(testInt);
    testDoublePub.set(testDouble);
    autoPub.set(SmartDashboard.getString("Auto Selector", "None"));
  }

  @Override
  public void teleopInit(){

  }

  @Override
  public void teleopPeriodic() {


    robotDrive.driveCartesian(yAxis, xAxis, zAxis);
    
    if (controller1.getAButton()==true) {
      shooterMotor.set(ControlMode.PercentOutput, -0.8);
      feederMotor.set(ControlMode.PercentOutput, -0.6);
      pivotMotor.set(ControlMode.MotionMagic, intakePosition);
      hoodServo.set(servoIntake);
    }
    else if (controller1.getBButton()==true && controller1.getRightBumper()==true) {
      shooterMotor.set(ControlMode.PercentOutput,1);
      feederMotor.set(ControlMode.PercentOutput,1);
      pivotMotor.set(ControlMode.MotionMagic, shootingHighPosition);
      hoodServo.set(servoDown);
    }
    else if (controller1.getBButton()==true){
      shooterMotor.set(ControlMode.PercentOutput,1);
      pivotMotor.set(ControlMode.MotionMagic, shootingHighPosition);
      hoodServo.set(servoDown);
    }
     else if (controller1.getXButton()==true && controller1.getRightBumper()==true) {
      shooterMotor.set(ControlMode.PercentOutput,0.6);
      feederMotor.set(ControlMode.PercentOutput, 0.7);
      pivotMotor.set(ControlMode.MotionMagic, shootingLowPosition);
      hoodServo.set(servoUp);
     }
     else if (controller1.getXButton()==true) {
      pivotMotor.set(ControlMode.MotionMagic, shootingLowPosition);
      hoodServo.set(servoUp);
     }
    else{
      feederMotor.set(ControlMode.PercentOutput,0);
      shooterMotor.set(ControlMode.PercentOutput,0);
      pivotMotor.set(ControlMode.MotionMagic, shootingHighPosition);
      if(pivotMotor.getSelectedSensorPosition() < shootingLowPosition)
        hoodServo.set(servoDown);
    }


  }

  @Override
  public void autonomousInit(){
      autoSelection = SmartDashboard.getString("Auto Selector", "None");
      timer.restart();
  }

  @Override
  public void autonomousPeriodic(){
    switch (autoSelection) {
      case "Do Nothing":
        turnOffAllMotors();
        break;
      case "Auto 1":
          if(timer.get() < 1)
            shooterMotor.set(ControlMode.PercentOutput, 0);
          else if(timer.get() < 2){
            feederMotor.set(ControlMode.PercentOutput, 1);
            testDouble = frontLeftEncoder.getPosition();
          }
          else if(timer.get() < 4){
            //Neo Encoder Units = Revs
            //GR = 10.71 - Wheel Circumference = 28.27in
            //2.63 Inches per Rev
            int position = 100;
            feederMotor.set(ControlMode.PercentOutput,0);
            shooterMotor.set(ControlMode.PercentOutput,0);
            frontLeftPID.setReference(position, CANSparkMax.ControlType.kPosition);
            rearLeftPID.setReference(position, CANSparkMax.ControlType.kPosition);
            frontRightPID.setReference(position, CANSparkMax.ControlType.kPosition);
            rearRightPID.setReference(position, CANSparkMax.ControlType.kPosition);
            testDouble = frontLeftEncoder.getPosition();
          }
        break;
      case "Auto 2" :
        if(timer.get() < 3){
          frontLeftPID.setReference(1000, CANSparkMax.ControlType.kPosition);
        }
        else if(timer.get() < 6){
          frontLeftPID.setReference(0, CANSparkMax.ControlType.kPosition);
        }
        else if(timer.get() < 9){
          frontLeftPID.setReference(1000, CANSparkMax.ControlType.kPosition);
        }
        break;
      case "Auto 3" :
        turnOffAllMotors();
        break;
      default:
        turnOffAllMotors();
        break;
    }
  }

  @Override
  public void disabledInit(){
    turnOffAllMotors();
  }

  @Override
  public void disabledPeriodic(){

  }

  public void turnOffAllMotors(){
    //Turn off all motors and do nothing
    frontLeft.set(0);
    rearLeft.set(0);
    rearRight.set(0);
    frontRight.set(0);
    shooterMotor.set(ControlMode.PercentOutput,0);
    feederMotor.set(ControlMode.PercentOutput, 0);
    pivotMotor.set(ControlMode.PercentOutput, 0);
  }

}

