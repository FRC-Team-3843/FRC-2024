// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.util.sendable.SendableRegistry;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {
  

  private MecanumDrive robotDrive;
  private XboxController controller1;

  private CANSparkMax frontLeft;
  private CANSparkMax rearLeft;
  private CANSparkMax frontRight;
  private CANSparkMax rearRight;

  private TalonSRX pivotMotor;
  private TalonSRX feederMotor;
  private TalonSRX shooterMotor;

  private double xAxis = 0;
  private double yAxis = 0;
  private double zAxis = 0;

  private String autoSelection;

  DoublePublisher xPub;
  DoublePublisher yPub;
  DoublePublisher zPub;
  StringPublisher auto;

  // Statics
  private static final int controllerChannel = 0;

  private static final int pivotMotorIdx = 0;
  private static final int pivotMotorTimeout = 30;

  private static final int shootingHighPosition = 5000;
  private static final int shootingLowPosition = 80000;
  private static final int intakePosition = 165000;

  private static final double deadband = 0.12;

  public static SendableChooser<String> autoChooser;

  
  private static final String[] autoList = {"Do Nothing", "Auto 1"};





  @Override
  public void robotInit() {
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
    auto = table.getStringTopic("Auto Selection").publish();



    frontLeft = new CANSparkMax(1,MotorType.kBrushless);
    rearLeft = new CANSparkMax(2,MotorType.kBrushless);
    rearRight = new CANSparkMax(3,MotorType.kBrushless);
    frontRight = new CANSparkMax(4,MotorType.kBrushless);
    
    pivotMotor = new TalonSRX(5);
    shooterMotor = new TalonSRX(6);
    feederMotor = new TalonSRX(7);

    //Set Factory Default
    pivotMotor.configFactoryDefault();
    //Set Sensor Feedback Device
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,pivotMotorIdx,pivotMotorTimeout);
    //Invert Sensor
    pivotMotor.setSensorPhase(true);
    //Invert Motor
    pivotMotor.setInverted(false);
    //Set Minimum Output
    pivotMotor.configNominalOutputForward(0);
    pivotMotor.configNominalOutputReverse(0);
    //Set Maximum Output
    pivotMotor.configPeakOutputForward(1);
    pivotMotor.configPeakOutputReverse(-1);
    //Set Allowable Error-Within this error motor will be in neutral
    pivotMotor.configAllowableClosedloopError(0, pivotMotorIdx, pivotMotorTimeout);
    //Set PID 
    pivotMotor.config_kF(pivotMotorIdx, 0.02325, pivotMotorTimeout);
		pivotMotor.config_kP(pivotMotorIdx, 2, pivotMotorTimeout);
		pivotMotor.config_kI(pivotMotorIdx, 0.001, pivotMotorTimeout);
		pivotMotor.config_kD(pivotMotorIdx, 20, pivotMotorTimeout);
    //Set acceleration and vcruise velocity
		pivotMotor.configMotionCruiseVelocity(25000, pivotMotorIdx);
		pivotMotor.configMotionAcceleration(25000, pivotMotorTimeout);
    //Set Smoothing
    pivotMotor.configMotionSCurveStrength(2);
    // Zero the sensor once on robot boot up
		pivotMotor.setSelectedSensorPosition(0, pivotMotorIdx, pivotMotorTimeout);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(false);
    rearRight.setInverted(false);
    frontLeft.setInverted(true);
    rearLeft.setInverted(true);

    feederMotor.setInverted(false);
    shooterMotor.setInverted(true);

    robotDrive = new MecanumDrive(frontLeft::set, rearLeft::set, frontRight::set, rearRight::set);

    controller1 = new XboxController(controllerChannel);

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
    auto.set(SmartDashboard.getString("Auto Selector", "None"));
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
    }
    else if (controller1.getBButton()==true && controller1.getRightBumper()==true) {
      shooterMotor.set(ControlMode.PercentOutput,1);
      feederMotor.set(ControlMode.PercentOutput,1);
      pivotMotor.set(ControlMode.MotionMagic, shootingHighPosition);
    }
    else if (controller1.getBButton()==true){
      shooterMotor.set(ControlMode.PercentOutput,1);
      pivotMotor.set(ControlMode.MotionMagic, shootingHighPosition);
    }
     else if (controller1.getXButton()==true && controller1.getRightBumper()==true) {
      shooterMotor.set(ControlMode.PercentOutput,0.6);
      feederMotor.set(ControlMode.PercentOutput, 0.7);
      pivotMotor.set(ControlMode.MotionMagic, shootingLowPosition);
     }
     else if (controller1.getXButton()==true) {
      pivotMotor.set(ControlMode.MotionMagic, shootingLowPosition);
     }
    else{
      feederMotor.set(ControlMode.PercentOutput,0);
      shooterMotor.set(ControlMode.PercentOutput,0);
      pivotMotor.set(ControlMode.MotionMagic, shootingHighPosition);
    }


  }

  @Override
  public void autonomousInit(){
      autoSelection = SmartDashboard.getString("Auto Selector", "None");
      System.out.println(autoSelection);
  }

  @Override
  public void autonomousPeriodic(){
    switch (SmartDashboard.getString("Auto Selector", "None")) {
      case "Do Nothing":
        turnOffAllMotors();
        break;
      case "Auto 1":
        shooterMotor.set(ControlMode.PercentOutput, .2);
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

