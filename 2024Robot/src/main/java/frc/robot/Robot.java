// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//update
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
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

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

public class Robot extends TimedRobot {
  
  private XboxController controller1, controller2;

  private TalonSRX pivotMotor, feederMotor, shieldMotor;

  private TalonFX shooterMotor;

  private final Timer timer = new Timer();
 
  private boolean[] autoSteps = new boolean[20];

  private double xAxis, yAxis, zAxis;

  private double previousEndTime, leftPosition, rightPosition;

  private String autoSelection;

  private MecanumDrive robotDrive;

  DoublePublisher xPub, yPub, zPub, testDoublePub;
  IntegerPublisher testIntPub;
  StringPublisher autoPub;

  
  @Override
  public void robotInit() {

    robotDrive = new MecanumDrive(Constants.frontLeftCanID, Constants.frontRightCanID, Constants.rearLeftCanID, Constants.rearRightCanID);
    robotDrive.setClosedLoop(Constants.driveP, Constants.driveI, Constants.driveD, Constants.driveIZ, Constants.driveFF);
    robotDrive.setSM(Constants.driveMaxVelocity, Constants.driveMaxAcceleration, Constants.driveAllowedError, Constants.driveMinVelocity);
    robotDrive.setMaxWheelVelocity(Constants.driveMaxWheelVelocity);
    robotDrive.setOutputRange(Constants.driveMaxReverseOutput, Constants.driveMaxForwardOutput);
    robotDrive.setRamp(Constants.driveRamp);
    robotDrive.resetEncoder();

    
    //Initialize Motor
    pivotMotor = new TalonSRX(Constants.pivotCanId);
    //Invert Motor
    pivotMotor.setInverted(Constants.pivotMotorInvert);
    //Set Sensor Feedback Device
    pivotMotor.configSelectedFeedbackSensor(Constants.pivotSensor);
    //Invert Sensor
    pivotMotor.setSensorPhase(Constants.pivotSensorInvert);
    //Set Maximum Output
    pivotMotor.configPeakOutputForward(Constants.pivotMaxForwardOutput);
    pivotMotor.configPeakOutputReverse(Constants.pivotMaxReverseOutput);
    //Set PID 
		pivotMotor.config_kP(0, Constants.pivotP);
		pivotMotor.config_kI(0, Constants.pivotI);
		pivotMotor.config_kD(0, Constants.pivotD);
    pivotMotor.config_kF(0, Constants.pivotFF);
    //Set acceleration and vcruise velocity
		pivotMotor.configMotionCruiseVelocity(Constants.pivotMaxVelocity);
		pivotMotor.configMotionAcceleration(Constants.pivotMaxAcceleration);
    //Set Smoothing
    pivotMotor.configMotionSCurveStrength(Constants.pivotCurve);
    // Zero the sensor once on robot boot up
		pivotMotor.setSelectedSensorPosition(0);

    //Initialize Motor
    shieldMotor = new TalonSRX(Constants.shieldCanId);
    //Invert Motor
    shieldMotor.setInverted(Constants.shieldMotorInvert);
    //Set Sensor Feedback Device
    shieldMotor.configSelectedFeedbackSensor(Constants.shieldSensor);
    //Invert Sensor
    shieldMotor.setSensorPhase(Constants.shieldSensorInvert);
    //Set Maximum Output
    shieldMotor.configPeakOutputForward(Constants.shieldMaxForwardOutput);
    shieldMotor.configPeakOutputReverse(Constants.shieldMaxReverseOutput);
    //Set PID 
		shieldMotor.config_kP(0, Constants.shieldP);
		shieldMotor.config_kI(0, Constants.shieldI);
		shieldMotor.config_kD(0, Constants.shieldD);
    shieldMotor.config_kF(0, Constants.shieldFF);
    // Zero the sensor once on robot boot up
		shieldMotor.setSelectedSensorPosition(Constants.shieldPos[1]);

    //Initialize Motor
    feederMotor = new TalonSRX(Constants.feederCanID);
    //Invert Motor
    feederMotor.setInverted(Constants.feedMotorInvert);
    
    //Initialize Motor
    shooterMotor = new TalonFX(Constants.shooterCanID);
    //Invert Motor
    shooterMotor.setInverted(Constants.shooterMotorInvert);

    //Setup camera
    CameraServer.startAutomaticCapture();
    
    //Setup Controllers
    controller1 = new XboxController(Constants.controller1Channel);
    controller2 = new XboxController(Constants.controller2Channel);

    //Add auto options to dashboard
    SmartDashboard.putStringArray("Auto List", Constants.autoList);  
    
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
    yAxis = MathUtil.applyDeadband(controller1.getLeftY(), (Constants.controllerDeadband*2));
    xAxis = -MathUtil.applyDeadband(controller1.getLeftX(), Constants.controllerDeadband);
    zAxis = -MathUtil.applyDeadband(controller1.getRightX(), Constants.controllerDeadband);
    
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
    
    robotDrive.drive(yAxis, xAxis, zAxis);
    
    if(controller1.getStartButtonPressed())
      robotDrive.invertVelocityMode();

    if(controller1.getBackButtonPressed())
      robotDrive.invertFieldCentric();

    if (controller1.getAButton()==true || controller2.getAButton()==true){  
      shooterMotor.set(-0.8);
      feederMotor.set(ControlMode.PercentOutput, -0.6);
      pivotMotor.set(ControlMode.MotionMagic, Constants.pivotPos[2]);
      if(pivotMotor.getSelectedSensorPosition() > Constants.pivotPos[1])
        shieldMotor.set(ControlMode.Position, Constants.shieldPos[0]);
    }
    else if ((controller1.getBButton()==true || controller2.getBButton()==true) && (controller1.getRightBumper()==true || controller2.getRightBumper()==true)){
      shooterMotor.set(1);
      feederMotor.set(ControlMode.PercentOutput,1);
      pivotMotor.set(ControlMode.MotionMagic, Constants.pivotPos[0]);
      shieldMotor.set(ControlMode.Position, Constants.shieldPos[1]);
    }
    else if (controller1.getBButton()==true || controller2.getBButton()==true){
      shooterMotor.set(1);
      pivotMotor.set(ControlMode.MotionMagic, Constants.pivotPos[0]);
      shieldMotor.set(ControlMode.Position, Constants.shieldPos[1]);
    }
    else if ((controller1.getXButton()==true || controller2.getXButton()==true) && (controller1.getRightBumper()==true || controller2.getRightBumper()==true)){
      shooterMotor.set(1);
      feederMotor.set(ControlMode.PercentOutput, 1);
      pivotMotor.set(ControlMode.MotionMagic, Constants.pivotPos[1]);
      shieldMotor.set(ControlMode.Position, Constants.shieldPos[2]);
     }
    else if (controller1.getXButton()==true || controller2.getXButton()==true){
      pivotMotor.set(ControlMode.MotionMagic, Constants.pivotPos[1]);
      shieldMotor.set(ControlMode.Position, Constants.shieldPos[2]);
     }
    else{
      feederMotor.set(ControlMode.PercentOutput,0);
      shooterMotor.set(0);
      pivotMotor.set(ControlMode.MotionMagic, Constants.pivotPos[0]);
      if(pivotMotor.getSelectedSensorPosition() < Constants.pivotPos[1])
        shieldMotor.set(ControlMode.Position, Constants.shieldPos[1]);
    }
  }

  @Override
  public void autonomousInit(){
    /*
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
      */
  }

  @Override
  public void autonomousPeriodic(){
    /*
    switch (autoSelection) {
      case "Do Nothing":
        turnOffAllMotors();
      break;
      
      case "Auto 1":
        //Just shoot
        if(!autoSteps[0]){
          pivotMotor.set(TalonSRXControlMode.MotionMagic, Constants.pivotPos[0]);
          shooterMotor.set(1);
          shieldMotor.set(ControlMode.Position, Constants.shieldPos[1]);
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
          pivotMotor.set(TalonSRXControlMode.MotionMagic, Constants.pivotPos[0]);
          shieldMotor.set(ControlMode.Position, Constants.shieldPos[1]);
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
          pivotMotor.set(TalonSRXControlMode.MotionMagic, Constants.pivotPos[2]);
          shieldMotor.set(ControlMode.Position, Constants.shieldPos[0]);
          if((Math.abs(pivotMotor.getSelectedSensorPosition() - Constants.pivotPos[2])) < 5000){
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
          pivotMotor.set(TalonSRXControlMode.MotionMagic, Constants.pivotPos[0]);
          if(pivotMotor.getSelectedSensorPosition() < Constants.pivotPos[1])
            shieldMotor.set(ControlMode.Position, Constants.shieldPos[1]);

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
          pivotMotor.set(TalonSRXControlMode.MotionMagic, Constants.pivotPos[0]);
          shieldMotor.set(ControlMode.Position, Constants.shieldPos[1]);
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
          pivotMotor.set(TalonSRXControlMode.MotionMagic, Constants.pivotPos[2]);
          shieldMotor.set(ControlMode.Position, Constants.shieldPos[0]);
          if((Math.abs(pivotMotor.getSelectedSensorPosition() - Constants.pivotPos[2])) < 5000){
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
          pivotMotor.set(TalonSRXControlMode.MotionMagic, Constants.pivotPos[0]);
          if(pivotMotor.getSelectedSensorPosition() < Constants.pivotPos[1])
            shieldMotor.set(ControlMode.Position, Constants.shieldPos[1]);
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
          pivotMotor.set(TalonSRXControlMode.MotionMagic, Constants.pivotPos[0]);
          shieldMotor.set(ControlMode.Position, Constants.shieldPos[1]);
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
          pivotMotor.set(TalonSRXControlMode.MotionMagic, Constants.pivotPos[2]);
          shieldMotor.set(ControlMode.Position, Constants.shieldPos[0]);
          if((Math.abs(pivotMotor.getSelectedSensorPosition() - Constants.pivotPos[2])) < 5000){
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
          pivotMotor.set(TalonSRXControlMode.MotionMagic, Constants.pivotPos[0]);
          if(pivotMotor.getSelectedSensorPosition() < Constants.pivotPos[1])
            shieldMotor.set(ControlMode.Position, Constants.shieldPos[1]);
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
          pivotMotor.set(TalonSRXControlMode.MotionMagic, Constants.pivotPos[0]);
          shieldMotor.set(ControlMode.Position, Constants.shieldPos[1]);
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
          pivotMotor.set(TalonSRXControlMode.MotionMagic, Constants.pivotPos[0]);
          shieldMotor.set(ControlMode.Position, Constants.shieldPos[1]);
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
    */
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
    /*
    frontLeft.set(0);
    rearLeft.set(0);
    rearRight.set(0);
    frontRight.set(0);
    shooterMotor.set(0);
    feederMotor.set(ControlMode.PercentOutput, 0);
    pivotMotor.set(ControlMode.PercentOutput, 0);
    */
  }

}

