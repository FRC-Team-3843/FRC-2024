// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//update
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

public class Robot extends TimedRobot {
  
  private XboxController controller1, controller2;

  private final Timer timer = new Timer();

  private CANSparkMax frontLeft, rearLeft, frontRight, rearRight;

  private SparkPIDController frontLeftPID, rearLeftPID, frontRightPID, rearRightPID;

  private RelativeEncoder frontLeftEncoder, rearLeftEncoder, frontRightEncoder, rearRightEncoder;

  private TalonSRX pivotMotor, feederMotor, shieldMotor;

  private TalonFX shooterMotor;

  private double xAxis = 0, yAxis = 0, zAxis = 0;

  private boolean velocityMode = false, fieldCentric = false;
  
  private double frontLeftOutput, rearLeftOutput, frontRightOutput, rearRightOutput, largestOutput;
 
  private boolean[] autoSteps = new boolean[20];

  private double previousEndTime, leftPosition, rightPosition;

  private String autoSelection;

  DoublePublisher xPub, yPub, zPub, testDoublePub;
  IntegerPublisher testIntPub;
  StringPublisher autoPub;

  int testInt = 0;
  double testDouble = 0;

  private static final int controllerChannel = 0;
  private static final int controller2Channel = 1;

  private static final int pivotMotorIdx = 0;
  private static final int pivotMotorTimeout = 30;

  private static final int shieldMotorIdx = 0;
  private static final int shieldMotorTimeout = 30;

  private static final int shootingHighPosition = 8000;
  private static final int shootingLowPosition = 73000;

  private static final int intakePosition = 160000;

  private static final double[] shieldPos = {0, 30, 60};

  private static final double deadband = 0.12;

  private static final int PIDSlot = 0;
  private static final double driveMotorP = .0001;
  private static final double driveMotorI = .000001;
  private static final double driveMotorKIz = 0;
  private static final double driveMotorD = 0;
  private static final double driveMotorFF = 0.000156;
  private static final double driveMotorMaxOutput = 1;
  private static final double driveMotorMinOutput = -1;
  private static final double driveMotorMaxVelocity = 5000; //rpm
  private static final double driveMotorMaxAcceleration = 8000;
  private static final double driveMotorMinVelocity = 0;
  private static final double driveMotorAllowedError = 0;

  private static final double ramp = 0.2;

  private static final double maxWheelVelocity = 5000;

  public static SendableChooser<String> autoChooser;

  private static final String[] autoList = {"Do Nothing", "Auto 1", "Auto 2", "Auto 3", "Auto 4", "Auto 5", "Auto 6"};


  @Override
  public void robotInit() {
    //Initialize Motors
    frontLeft = new CANSparkMax(1,MotorType.kBrushless);
    rearLeft = new CANSparkMax(2,MotorType.kBrushless);
    frontRight = new CANSparkMax(3,MotorType.kBrushless);
    rearRight = new CANSparkMax(4,MotorType.kBrushless);
    
    pivotMotor = new TalonSRX(5);
    shieldMotor = new TalonSRX(6);
    feederMotor = new TalonSRX(7);
    
    shooterMotor = new TalonFX(8);



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
		pivotMotor.config_kD(pivotMotorIdx, 2, pivotMotorTimeout);
    //Set acceleration and vcruise velocity
		pivotMotor.configMotionCruiseVelocity(100000, pivotMotorIdx);
		pivotMotor.configMotionAcceleration(60000, pivotMotorTimeout);
    //Set Smoothing
    pivotMotor.configMotionSCurveStrength(0);
    // Zero the sensor once on robot boot up
		pivotMotor.setSelectedSensorPosition(0, pivotMotorIdx, pivotMotorTimeout);

    //Setup Shield Motor
    shieldMotor.configFactoryDefault();
    //Set Sensor Feedback Device
    shieldMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,shieldMotorIdx,shieldMotorTimeout);
    //Invert Sensor
    shieldMotor.setSensorPhase(false);
    //Invert Motor
    shieldMotor.setInverted(false);
    //Set Maximum Output
    shieldMotor.configPeakOutputForward(.6);
    shieldMotor.configPeakOutputReverse(-.6);
    //Set PID 
    shieldMotor.config_kF(shieldMotorIdx, 0, shieldMotorTimeout);
		shieldMotor.config_kP(shieldMotorIdx, 80, shieldMotorTimeout);
		shieldMotor.config_kI(shieldMotorIdx, 0, shieldMotorTimeout);
		shieldMotor.config_kD(shieldMotorIdx, 0, shieldMotorTimeout);
    // Zero the sensor once on robot boot up
		shieldMotor.setSelectedSensorPosition(30, shieldMotorIdx, shieldMotorTimeout);

    // Setup drive motors
    //frontRight.restoreFactoryDefaults();
    frontRight.setInverted(false);
    frontRight.setOpenLoopRampRate(ramp);
    frontRightPID = frontRight.getPIDController();
    frontRightEncoder = frontRight.getEncoder();
    frontRightPID.setP(driveMotorP);
    frontRightPID.setI(driveMotorI);
    frontRightPID.setIZone(driveMotorKIz);
    frontRightPID.setD(driveMotorD);
    frontRightPID.setFF(driveMotorFF);
    frontRightPID.setOutputRange(driveMotorMinOutput, driveMotorMaxOutput);
    frontRightPID.setSmartMotionMaxVelocity(driveMotorMaxVelocity, PIDSlot);
    frontRightPID.setSmartMotionMaxAccel(driveMotorMaxAcceleration, PIDSlot);
    frontRightPID.setSmartMotionAllowedClosedLoopError(driveMotorAllowedError, PIDSlot);
    frontRightPID.setSmartMotionMinOutputVelocity(driveMotorMinVelocity, PIDSlot);
    
    rearRight.setInverted(false);
    rearRight.setOpenLoopRampRate(ramp);
    rearRightPID = rearRight.getPIDController();
    rearRightEncoder = rearRight.getEncoder();
    rearRightPID.setP(driveMotorP);
    rearRightPID.setI(driveMotorI);
    rearRightPID.setIZone(driveMotorKIz);
    rearRightPID.setD(driveMotorD);
    rearRightPID.setFF(driveMotorFF);
    rearRightPID.setOutputRange(driveMotorMinOutput, driveMotorMaxOutput);
    rearRightPID.setSmartMotionMaxVelocity(driveMotorMaxVelocity, PIDSlot);
    rearRightPID.setSmartMotionMaxAccel(driveMotorMaxAcceleration, PIDSlot);
    rearRightPID.setSmartMotionAllowedClosedLoopError(driveMotorAllowedError, PIDSlot);
    rearRightPID.setSmartMotionMinOutputVelocity(driveMotorMinVelocity, PIDSlot);

    frontLeft.setInverted(true);
    frontLeft.setOpenLoopRampRate(ramp);
    frontLeftPID = frontLeft.getPIDController();
    frontLeftEncoder = frontLeft.getEncoder();
    frontLeftPID.setP(driveMotorP);
    frontLeftPID.setI(driveMotorI);
    frontLeftPID.setIZone(driveMotorKIz);
    frontLeftPID.setD(driveMotorD);
    frontLeftPID.setFF(driveMotorFF);
    frontLeftPID.setOutputRange(driveMotorMinOutput, driveMotorMaxOutput);
    frontLeftPID.setSmartMotionMaxVelocity(driveMotorMaxVelocity, PIDSlot);
    frontLeftPID.setSmartMotionMaxAccel(driveMotorMaxAcceleration, PIDSlot);
    frontLeftPID.setSmartMotionAllowedClosedLoopError(driveMotorAllowedError, PIDSlot);
    frontLeftPID.setSmartMotionMinOutputVelocity(driveMotorMinVelocity, PIDSlot);

    rearLeft.setInverted(true);
    rearLeft.setOpenLoopRampRate(ramp);
    rearLeftPID = rearLeft.getPIDController();
    rearLeftEncoder = rearLeft.getEncoder();
    rearLeftPID.setP(driveMotorP);
    rearLeftPID.setI(driveMotorI);
    rearLeftPID.setIZone(driveMotorKIz);
    rearLeftPID.setD(driveMotorD);
    rearLeftPID.setFF(driveMotorFF);
    rearLeftPID.setOutputRange(driveMotorMinOutput, driveMotorMaxOutput);
    rearLeftPID.setSmartMotionMaxVelocity(driveMotorMaxVelocity, PIDSlot);
    rearLeftPID.setSmartMotionMaxAccel(driveMotorMaxAcceleration, PIDSlot);
    rearLeftPID.setSmartMotionAllowedClosedLoopError(driveMotorAllowedError, PIDSlot);
    rearLeftPID.setSmartMotionMinOutputVelocity(driveMotorMinVelocity, PIDSlot);

    //Setup feeder and shoorter motors
    feederMotor.setInverted(false);
    shooterMotor.setInverted(true);

    //Setup camera
    CameraServer.startAutomaticCapture();
    
    //Setup Controllers
    controller1 = new XboxController(controllerChannel);
    controller2 = new XboxController(controller2Channel);

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

    autoPub.set(SmartDashboard.getString("Auto Selector", "None"));    

  }

  @Override
  public void robotPeriodic(){
    // Use the joystick Y axis for forward movement, X axis for lateral
    // movement, and Z axis for rotation.
    yAxis = MathUtil.applyDeadband(controller1.getLeftY(), (deadband*2));
    xAxis = -MathUtil.applyDeadband(controller1.getLeftX(), deadband);
    zAxis = -MathUtil.applyDeadband(controller1.getRightX(), deadband);
    
    xPub.set(xAxis);
    yPub.set(yAxis);
    zPub.set(zAxis);
  }

  @Override
  public void teleopInit(){
    //robotDrive = new MecanumDrive(frontLeft::set, rearLeft::set, frontRight::set, rearRight::set);
  }

  @Override
  public void teleopPeriodic() {

    //robotDrive.driveCartesian(yAxis, xAxis, zAxis);
    frontLeftOutput = yAxis + xAxis + zAxis;
    frontRightOutput = yAxis - xAxis - zAxis;
    rearLeftOutput = yAxis - xAxis + zAxis;
    rearRightOutput = yAxis + xAxis - zAxis;

    largestOutput = Math.abs(frontLeftOutput);
    if (largestOutput <  Math.abs(frontRightOutput)){
      largestOutput = frontRightOutput;
    }
    if (largestOutput < Math.abs(rearLeftOutput)){
      largestOutput = rearLeftOutput;
    }
    if (largestOutput < Math.abs(rearRightOutput)) {
      largestOutput = rearRightOutput;
    }
    if (largestOutput > 1) {
      frontLeftOutput = frontLeftOutput / largestOutput;
      frontRightOutput = frontRightOutput / largestOutput;
      rearLeftOutput = rearLeftOutput / largestOutput;
      rearRightOutput = rearRightOutput / largestOutput;
    }



    if(velocityMode){
      frontLeftPID.setReference(maxWheelVelocity * frontLeftOutput, ControlType.kSmartVelocity);
      frontRightPID.setReference(maxWheelVelocity * frontRightOutput, ControlType.kSmartVelocity);
      rearLeftPID.setReference(maxWheelVelocity * rearLeftOutput, ControlType.kSmartVelocity);
      rearRightPID.setReference(maxWheelVelocity * rearRightOutput, ControlType.kSmartVelocity);
    }  
    else{
      frontLeft.set(frontLeftOutput);
      frontRight.set(frontRightOutput);
      rearLeft.set(rearLeftOutput);
      rearRight.set(rearRightOutput);
    }
      
    if(controller1.getStartButtonPressed())
      velocityMode = !velocityMode;

    if(controller1.getBackButtonPressed())
      fieldCentric = !fieldCentric;

    if (controller1.getAButton()==true || controller2.getAButton()==true){  
      shooterMotor.set(-0.8);
      feederMotor.set(ControlMode.PercentOutput, -0.6);
      pivotMotor.set(ControlMode.MotionMagic, intakePosition);
      if(pivotMotor.getSelectedSensorPosition() > shootingLowPosition)
        shieldMotor.set(ControlMode.Position, shieldPos[0]);
    }
    else if ((controller1.getBButton()==true || controller2.getBButton()==true) && (controller1.getRightBumper()==true || controller2.getRightBumper()==true)){
      shooterMotor.set(1);
      feederMotor.set(ControlMode.PercentOutput,1);
      pivotMotor.set(ControlMode.MotionMagic, shootingHighPosition);
      shieldMotor.set(ControlMode.Position, shieldPos[1]);
    }
    else if (controller1.getBButton()==true || controller2.getBButton()==true){
      shooterMotor.set(1);
      pivotMotor.set(ControlMode.MotionMagic, shootingHighPosition);
      shieldMotor.set(ControlMode.Position, shieldPos[1]);
    }
    else if ((controller1.getXButton()==true || controller2.getXButton()==true) && (controller1.getRightBumper()==true || controller2.getRightBumper()==true)){
      shooterMotor.set(1);
      feederMotor.set(ControlMode.PercentOutput, 1);
      pivotMotor.set(ControlMode.MotionMagic, shootingLowPosition);
      shieldMotor.set(ControlMode.Position, shieldPos[2]);
     }
    else if (controller1.getXButton()==true || controller2.getXButton()==true){
      pivotMotor.set(ControlMode.MotionMagic, shootingLowPosition);
      shieldMotor.set(ControlMode.Position, shieldPos[2]);
     }
    else{
      feederMotor.set(ControlMode.PercentOutput,0);
      shooterMotor.set(0);
      pivotMotor.set(ControlMode.MotionMagic, shootingHighPosition);
      if(pivotMotor.getSelectedSensorPosition() < shootingLowPosition)
        shieldMotor.set(ControlMode.Position, shieldPos[1]);
    }
  }

  @Override
  public void autonomousInit(){
      autoSelection = SmartDashboard.getString("Auto Selector", "None");

      //Neo Encoder Units = Revs
      //GR = 10.71 - Wheel Circumference = 28.27in
      //2.63 Inches per Rev
      frontLeftEncoder.setPosition(0);
      rearLeftEncoder.setPosition(0);
      frontRightEncoder.setPosition(0);
      rearRightEncoder.setPosition(0);
      leftPosition = 0;
      rightPosition = 0;

      timer.restart();
      previousEndTime = timer.get();

      for(int i = 0; i < 20; i++){
        autoSteps[i] = false;
      }
  }

  @Override
  public void autonomousPeriodic(){
    switch (autoSelection) {
      case "Do Nothing":
        turnOffAllMotors();
      break;
      
      case "Auto 1":
        //Just shoot
        if(!autoSteps[0]){
          pivotMotor.set(TalonSRXControlMode.MotionMagic, shootingHighPosition);
          shooterMotor.set(1);
          shieldMotor.set(ControlMode.Position, shieldPos[1]);
          if(timer.get() > previousEndTime + 1){
            autoSteps[0] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[1]){
          feederMotor.set(ControlMode.PercentOutput, 1);
          if(timer.get() > previousEndTime + 0.5){
            autoSteps[1] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else
          turnOffAllMotors();
          
        break;
      
        case "Auto 2" :
        // Center field double auto
        if(!autoSteps[0]){
          shooterMotor.set(1);
          pivotMotor.set(TalonSRXControlMode.MotionMagic, shootingHighPosition);
          shieldMotor.set(ControlMode.Position, shieldPos[1]);
          if(timer.get() > previousEndTime + 1){
            autoSteps[0] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[1]){
          feederMotor.set(ControlMode.PercentOutput, 1);
          if(timer.get() > previousEndTime + 0.5){
            autoSteps[1] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[2]){
          shooterMotor.set(0);
          feederMotor.set(ControlMode.PercentOutput, 0);
          pivotMotor.set(TalonSRXControlMode.MotionMagic, intakePosition);
          shieldMotor.set(ControlMode.Position, shieldPos[0]);
          if((Math.abs(pivotMotor.getSelectedSensorPosition() - intakePosition)) < 5000){
            autoSteps[2] = true;
            previousEndTime = timer.get();
            leftPosition += -25;
            rightPosition += -25;
          }
        }
        else if(!autoSteps[3]){
          shooterMotor.set(-0.8);
          feederMotor.set(ControlMode.PercentOutput, -0.6);
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[3] = true;
            previousEndTime = timer.get();
            leftPosition += 25;
            rightPosition += 25;
          }
        }
        else if(!autoSteps[5]){
          shooterMotor.set(0);
          feederMotor.set(ControlMode.PercentOutput, 0);
          pivotMotor.set(TalonSRXControlMode.MotionMagic, shootingHighPosition);
          if(pivotMotor.getSelectedSensorPosition() < shootingLowPosition)
            shieldMotor.set(ControlMode.Position, shieldPos[1]);

          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[5] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[6]){
          shooterMotor.set(1);
          if(timer.get() > previousEndTime + 1){
            autoSteps[6] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[7]){
          feederMotor.set(ControlMode.PercentOutput, 1);
          if(timer.get() > previousEndTime + 0.5){
            autoSteps[7] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else
          turnOffAllMotors();
        break;
      
        case "Auto 3":
        //Right Side Facing Speaker from midfield
        if(!autoSteps[0]){
          shooterMotor.set(1);
          pivotMotor.set(TalonSRXControlMode.MotionMagic, shootingHighPosition);
          shieldMotor.set(ControlMode.Position, shieldPos[1]);
          if(timer.get() > previousEndTime + 1){
            autoSteps[0] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[1]){
          feederMotor.set(ControlMode.PercentOutput, 1);
          if(timer.get() > previousEndTime + 0.5){
            autoSteps[1] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[2]){
          shooterMotor.set(0);
          feederMotor.set(ControlMode.PercentOutput, 0);
          pivotMotor.set(TalonSRXControlMode.MotionMagic, intakePosition);
          shieldMotor.set(ControlMode.Position, shieldPos[0]);
          if((Math.abs(pivotMotor.getSelectedSensorPosition() - intakePosition)) < 5000){
            autoSteps[2] = true;
            previousEndTime = timer.get();
            leftPosition += -15;
            rightPosition += -15;
          }
        }
        else if(!autoSteps[3]){
          shooterMotor.set(-0.8);
          feederMotor.set(ControlMode.PercentOutput, -0.6);
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[3] = true;
            previousEndTime = timer.get();
            leftPosition += -30;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[4]){
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[4] = true;
            previousEndTime = timer.get();
            leftPosition += -20;
            rightPosition += -20;
          }
        }
        else if(!autoSteps[5]){
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[5] = true;
            previousEndTime = timer.get();
            leftPosition += 20;
            rightPosition += 20;
          }
        }
        else if(!autoSteps[6]){
          pivotMotor.set(TalonSRXControlMode.MotionMagic, shootingHighPosition);
          if(pivotMotor.getSelectedSensorPosition() < shootingLowPosition)
            shieldMotor.set(ControlMode.Position, shieldPos[1]);
          shooterMotor.set(0);
          feederMotor.set(ControlMode.PercentOutput, 0);
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[6] = true;
            previousEndTime = timer.get();
            leftPosition += 30;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[7]){
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[7] = true;
            previousEndTime = timer.get();
            leftPosition += 15;
            rightPosition += 15;
          }
        }
        else if(!autoSteps[8]){
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[8] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[9]){
          shooterMotor.set(1);
          if(timer.get() > previousEndTime + 1){
            autoSteps[9] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[10]){
          feederMotor.set(ControlMode.PercentOutput, 1);
          if(timer.get() > previousEndTime + 0.5){
            autoSteps[10] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else
          turnOffAllMotors();
        break;
      case "Auto 4":
        //Left Side Facing Speaker from midfield
        if(!autoSteps[0]){
          shooterMotor.set(1);
          pivotMotor.set(TalonSRXControlMode.MotionMagic, shootingHighPosition);
          shieldMotor.set(ControlMode.Position, shieldPos[1]);
          if(timer.get() > previousEndTime + 1){
            autoSteps[0] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[1]){
          feederMotor.set(ControlMode.PercentOutput, 1);
          if(timer.get() > previousEndTime + 0.5){
            autoSteps[1] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[2]){
          shooterMotor.set(0);
          feederMotor.set(ControlMode.PercentOutput, 0);
          pivotMotor.set(TalonSRXControlMode.MotionMagic, intakePosition);
          shieldMotor.set(ControlMode.Position, shieldPos[0]);
          if((Math.abs(pivotMotor.getSelectedSensorPosition() - intakePosition)) < 5000){
            autoSteps[2] = true;
            previousEndTime = timer.get();
            leftPosition += -15;
            rightPosition += -15;
          }
        }
        else if(!autoSteps[3]){
          shooterMotor.set(-0.8);
          feederMotor.set(ControlMode.PercentOutput, -0.6);
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[3] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += -30;
          }
        }
        else if(!autoSteps[4]){
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[4] = true;
            previousEndTime = timer.get();
            leftPosition += -20;
            rightPosition += -20;
          }
        }
        else if(!autoSteps[5]){
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[5] = true;
            previousEndTime = timer.get();
            leftPosition += 20;
            rightPosition += 20;
          }
        }
        else if(!autoSteps[6]){
          pivotMotor.set(TalonSRXControlMode.MotionMagic, shootingHighPosition);
          if(pivotMotor.getSelectedSensorPosition() < shootingLowPosition)
            shieldMotor.set(ControlMode.Position, shieldPos[1]);
          shooterMotor.set(0);
          feederMotor.set(ControlMode.PercentOutput, 0);
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[6] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 30;
          }
        }
        else if(!autoSteps[7]){
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[7] = true;
            previousEndTime = timer.get();
            leftPosition += 15;
            rightPosition += 15;
          }
        }
        else if(!autoSteps[8]){
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[8] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[9]){
          shooterMotor.set(1);
          if(timer.get() > previousEndTime + 1){
            autoSteps[9] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[10]){
          feederMotor.set(ControlMode.PercentOutput, 1);
          if(timer.get() > previousEndTime + 0.5){
            autoSteps[10] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else
          turnOffAllMotors();
        break;
      case "Auto 5":
        //Right Side Facing Speaker from midfield
        if(!autoSteps[0]){
          shooterMotor.set(1);
          pivotMotor.set(TalonSRXControlMode.MotionMagic, shootingHighPosition);
          shieldMotor.set(ControlMode.Position, shieldPos[1]);
          if(timer.get() > previousEndTime + 1){
            autoSteps[0] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[1]){
          feederMotor.set(ControlMode.PercentOutput, 1);
          if(timer.get() > previousEndTime + 0.5){
            autoSteps[1] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[2]){
          feederMotor.set(ControlMode.PercentOutput, 0);
          shooterMotor.set(0);
          if(timer.get() > previousEndTime + 10){
            autoSteps[2] = true;
            previousEndTime = timer.get();
            leftPosition += -47;
            rightPosition += -15;
          }
        }
        else if(!autoSteps[3]){
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[3] = true;
            previousEndTime = timer.get();
            leftPosition += -15;
            rightPosition += -15;
          }
        }
        else if(!autoSteps[4]){
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[4] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else
          turnOffAllMotors();
        break;
      case "Auto 6":
        //Left Side Facing Speaker from midfield
        if(!autoSteps[0]){
          shooterMotor.set(1);
          pivotMotor.set(TalonSRXControlMode.MotionMagic, shootingHighPosition);
          shieldMotor.set(ControlMode.Position, shieldPos[1]);
          if(timer.get() > previousEndTime + 1){
            autoSteps[0] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[1]){
          feederMotor.set(ControlMode.PercentOutput, 1);
          if(timer.get() > previousEndTime + 0.5){
            autoSteps[1] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else if(!autoSteps[2]){
          feederMotor.set(ControlMode.PercentOutput, 0);
          shooterMotor.set(0);
          if(timer.get() > previousEndTime + 10){
            autoSteps[2] = true;
            previousEndTime = timer.get();
            leftPosition += -15;
            rightPosition += -47;
          }
        }
        else if(!autoSteps[3]){
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[3] = true;
            previousEndTime = timer.get();
            leftPosition += -15;
            rightPosition += -15;
          }
        }
        else if(!autoSteps[4]){
          frontLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          rearLeftPID.setReference(leftPosition, CANSparkMax.ControlType.kSmartMotion);
          frontRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          rearRightPID.setReference(rightPosition, CANSparkMax.ControlType.kSmartMotion);
          if(((Math.abs(frontLeftEncoder.getPosition() - leftPosition)) < 0.5) &&  
          ((Math.abs(frontRightEncoder.getPosition() - rightPosition)) < 0.5)){
            autoSteps[4] = true;
            previousEndTime = timer.get();
            leftPosition += 0;
            rightPosition += 0;
          }
        }
        else
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
    shooterMotor.set(0);
    feederMotor.set(ControlMode.PercentOutput, 0);
    pivotMotor.set(ControlMode.PercentOutput, 0);
  }

}

