package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;


public class MecanumDrive {
    private int frontLeftCanID, frontRightCanID, rearLeftCanID, rearRightCanID;

    private List<CANSparkMax> motors = new ArrayList<>();

    private double maxWheelVelocity = 0;

    private boolean velocityMode = false, fieldCentric = false;
    
    public MecanumDrive(int frontLeftCanID, int frontRightCanID, int rearLeftCanID, int rearRightCanID){
        this.frontLeftCanID = frontLeftCanID;
        this.frontRightCanID = frontRightCanID;
        this.rearLeftCanID = rearLeftCanID;
        this.rearRightCanID = rearRightCanID;

        motors.add(new CANSparkMax(frontLeftCanID, MotorType.kBrushless));
        motors.add(new CANSparkMax(frontRightCanID, MotorType.kBrushless));
        motors.add(new CANSparkMax(rearLeftCanID, MotorType.kBrushless));
        motors.add(new CANSparkMax(rearRightCanID, MotorType.kBrushless));

        motors.stream().forEach(motor -> motor.setInverted(false));
    }

    private <T> T getProperty(int canID, Function<CANSparkMax, T> extractor) {
        return motors.stream()
                     .filter(motor -> motor.getDeviceId() == canID)
                     .findFirst()
                     .map(extractor)
                     .orElseThrow(() -> new RuntimeException("Device not found"));
    }

    public void invertMotor (int canId){
        CANSparkMax motor = getProperty(canId, Function.identity());
        motor.setInverted(!motor.getInverted());
    }

    public void invertMotor (int canID, boolean invert){
        getProperty(canID, Function.identity()).setInverted(invert);
    }

    public void setRamp (double ramp){
        motors.stream().forEach(motor -> motor.setOpenLoopRampRate(ramp));
    }

    public void setRamp (double ramp, int canID){
        getProperty(canID, Function.identity()).setOpenLoopRampRate(ramp);
    }

    public void setP (double P){
        motors.stream().forEach(motor -> motor.getPIDController().setP(P));
    }

    public void setP (double P, int canID){
        getProperty(canID, CANSparkMax::getPIDController).setP(P);
    }

    public void setI (double I){
        motors.stream().forEach(motor -> motor.getPIDController().setI(I));
    }

    public void setI (double I, int canID){
        getProperty(canID, CANSparkMax::getPIDController).setI(I);
    }

    public void setD (double D){
        motors.stream().forEach(motor -> motor.getPIDController().setD(D));
    }

    public void setD (double D, int canID){
        getProperty(canID, CANSparkMax::getPIDController).setD(D);
    }

    public void setFF (double FF){
        motors.stream().forEach(motor -> motor.getPIDController().setFF(FF));
    }

    public void setFF (double FF, int canID){
        getProperty(canID, CANSparkMax::getPIDController).setFF(FF);
    }

    public void setIZone (double IZ){
        motors.stream().forEach(motor -> motor.getPIDController().setIZone(IZ));
    }

    public void setIZone (double IZ, int canID){
        getProperty(canID, CANSparkMax::getPIDController).setIZone(IZ);
    }
    
    public void setOutputRange (double forwardMax, double reverseMax){
        motors.stream().forEach(motor -> motor.getPIDController().setOutputRange(forwardMax, reverseMax));
    }

    public void setOutputRange (double forwardMax, double reverseMax, int canID){
        getProperty(canID, CANSparkMax::getPIDController).setOutputRange(forwardMax, reverseMax);
    }

    public void setSMMaxVelocity (double maxVelocity){
        setSMMaxVelocity(maxVelocity, 0);
    }

    public void setSMMaxVelocity (double maxVelocity, int slot){
        motors.stream().forEach(motor -> motor.getPIDController().setSmartMotionMaxVelocity(maxVelocity, slot));
    }

    public void setSMMaxVelocity (double maxVelocity, int slot, int canID){
        getProperty(canID, CANSparkMax::getPIDController).setSmartMotionMaxVelocity(maxVelocity, slot);
    }

    public void setSMAccel (double maxAccel){
        setSMAccel(maxAccel, 0);
    }

    public void setSMAccel (double maxAccel, int slot){
        motors.stream().forEach(motor -> motor.getPIDController().setSmartMotionMaxAccel(maxAccel, slot));
    }

    public void setSMAccel (double maxAccel, int slot, int canID){
        getProperty(canID, CANSparkMax::getPIDController).setSmartMotionMaxAccel(maxAccel, slot);
    }

    public void setSMAllowedError (double allowedError){
        setSMAllowedError(allowedError, 0);
    }

    public void setSMAllowedError (double allowedError, int slot){
        motors.stream().forEach(motor -> motor.getPIDController().setSmartMotionAllowedClosedLoopError(allowedError, slot));
    }

    public void setSMAllowedError (double allowedError, int slot, int canID){
        getProperty(canID, CANSparkMax::getPIDController).setSmartMotionAllowedClosedLoopError(allowedError, slot);
    }
    
    public void setSMMinVelocity (double minVelocity){
        setSMMinVelocity(minVelocity, 0);
    }

    public void setSMMinVelocity (double minVelocity, int slot){
        motors.stream().forEach(motor -> motor.getPIDController().setSmartMotionMinOutputVelocity(minVelocity, slot));
    }

    public void setSMMinVelocity (double minVelocity, int slot, int canID){
        getProperty(canID, CANSparkMax::getPIDController).setSmartMotionMinOutputVelocity(minVelocity, slot);
    }

    public void setMaxWheelVelocity (double maxWheelVelocity){
        this.maxWheelVelocity = maxWheelVelocity;
    }

    public void stopMotor(){
        motors.stream().forEach(motor -> motor.stopMotor());
    }
    
    public void stopMotor(int canID){
        getProperty(canID, Function.identity()).stopMotor();
    }

    public void resetEncoder(){
        motors.stream().forEach(motor -> motor.getEncoder().setPosition(0));
    }

    public void resetEncoder(int canID){
        getProperty(canID, CANSparkMax::getEncoder).setPosition(0);
    }

    public void setEncoder(int position){
        motors.stream().forEach(motor -> motor.getEncoder().setPosition(position));
    }

    public void setEncoder(int position, int canID){
        getProperty(canID, CANSparkMax::getEncoder).setPosition(position);
    }

    public void setClosedLoop(double P, double I, double D, double IZ, double FF){
        setP(P);
        setI(I);
        setD(D);
        setIZone(IZ);
        setFF(FF);
    }

    public void setClosedLoop(double P, double I, double D, double IZ, double FF, int canID){
        setP(P, canID);
        setI(I, canID);
        setD(D, canID);
        setIZone(IZ, canID);
        setFF(FF, canID);
    }

    public void setSM(double maxVelocity, double maxAccel, double allowedError, double minVelocity){
        setSMMaxVelocity(maxVelocity);
        setSMAccel(maxAccel);
        setSMAllowedError(allowedError);
        setSMMinVelocity(minVelocity);
    }

    public void setSM(double maxVelocity, double maxAccel, double allowedError, double minVelocity, int slot){
        setSMMaxVelocity(maxVelocity, slot);
        setSMAccel(maxAccel, slot);
        setSMAllowedError(allowedError, slot);
        setSMMinVelocity(minVelocity, slot);
    }

    public void setSM(double maxVelocity, double maxAccel, double allowedError, double minVelocity, int slot, int canID){
        setSMMaxVelocity(maxVelocity, slot, canID);
        setSMAccel(maxAccel, slot, canID);
        setSMAllowedError(allowedError, slot, canID);
        setSMMinVelocity(minVelocity, slot, canID);
    }

    public void setVelocityMode(boolean setVelocityMode){
        this.velocityMode = setVelocityMode;
    }

    public void invertVelocityMode(){
        velocityMode = !velocityMode;
    }

    public void setFieldCentric(boolean setFieldCentric){
        this.fieldCentric = setFieldCentric;
    }
    
    public void invertFieldCentric(){
        fieldCentric = !fieldCentric;
    }

    public void drive(double yAxis, double xAxis, double zAxis) {
        List<Double> outputs = calculateOutputs(yAxis, xAxis, zAxis);
        double largestOutput = calculateLargestOutput(outputs);
    
        if (largestOutput > 1)
            normalizeOutputs(outputs, largestOutput);

        if(velocityMode)
            setMotorPercentVelocity(outputs.get(0), outputs.get(1), outputs.get(2), outputs.get(3));
        else
            setMotorVOutputs(outputs.get(0), outputs.get(1), outputs.get(2), outputs.get(3));

    }
    
    public void drive(double yAxis, double xAxis, double zAxis, double gyroAngle) {
        if(!fieldCentric){
            drive(yAxis, xAxis, zAxis);
            return;
        }
        else{
            // Perform rotation based on gyro angle
            double angleRad = Math.toRadians(gyroAngle);
            double cosAngle = Math.cos(angleRad);
            double sinAngle = Math.sin(angleRad);
            double tempXAxis = xAxis * cosAngle - yAxis * sinAngle;
            double tempYAxis = xAxis * sinAngle + yAxis * cosAngle;
        
            // Call the original drive method with the rotated axes
            drive(tempYAxis, tempXAxis, zAxis);
        }
    }

    private List<Double> calculateOutputs(double yAxis, double xAxis, double zAxis) {
        List<Double> outputs = new ArrayList<>();
        outputs.add(yAxis + xAxis + zAxis); // Front left
        outputs.add(yAxis - xAxis - zAxis); // Front right
        outputs.add(yAxis - xAxis + zAxis); // Rear left
        outputs.add(yAxis + xAxis - zAxis); // Rear right
        return outputs;
    }
    
    private double calculateLargestOutput(List<Double> outputs) {
        return outputs.stream()
                      .map(Math::abs)
                      .max(Double::compare)
                      .orElse(0.0);
    }
    
    private void normalizeOutputs(List<Double> outputs, double largestOutput) {
        for (int i = 0; i < outputs.size(); i++) {
            outputs.set(i, outputs.get(i) / largestOutput);
        }
    }

    public void setMotorVOutputs(double frontLeft, double frontRight, double rearLeft, double rearRight){
        setFrontLeftMotorVOutput(frontLeft);
        setFrontRightMotorVOutput(frontRight);
        setRearLeftMotorVOutput(rearLeft);
        setRearRightMotorVOutput(rearRight);
    }

    public void setMotorPercentVelocity(double frontLeft, double frontRight, double rearLeft, double rearRight){
        setFrontLeftMotorVelocity(frontLeft * maxWheelVelocity);
        setFrontRightMotorVelocity(frontRight * maxWheelVelocity);
        setRearLeftMotorVelocity(rearLeft * maxWheelVelocity);
        setRearRightMotorVelocity(rearRight * maxWheelVelocity);
    }

    public void setMotorVelocity(double frontLeft, double frontRight, double rearLeft, double rearRight){
        setFrontLeftMotorVelocity(frontLeft);
        setFrontRightMotorVelocity(frontRight);
        setRearLeftMotorVelocity(rearLeft);
        setRearRightMotorVelocity(rearRight);
    }

    public void setFrontLeftMotorVOutput (double vOutput){
        getProperty(frontLeftCanID, Function.identity()).set(vOutput);
    }

    public void setFrontRightMotorVOutput (double vOutput){
        getProperty(frontRightCanID, Function.identity()).set(vOutput);
    }

    public void setRearLeftMotorVOutput (double vOutput){
        getProperty(rearLeftCanID, Function.identity()).set(vOutput);
    }

    public void setRearRightMotorVOutput (double vOutput){
        getProperty(rearRightCanID, Function.identity()).set(vOutput);
    }

    public void setFrontLeftMotorVelocity (double velocity){
       getProperty(frontLeftCanID, motor -> motor.getPIDController().setReference(velocity, ControlType.kSmartVelocity));
    }

    public void setFrontRightMotorVelocity(double velocity) {
        getProperty(frontRightCanID, motor -> motor.getPIDController().setReference(velocity, ControlType.kSmartVelocity));
    }
    
    public void setRearLeftMotorVelocity(double velocity) {
        getProperty(rearLeftCanID, motor -> motor.getPIDController().setReference(velocity, ControlType.kSmartVelocity));
    }
    
    public void setRearRightMotorVelocity(double velocity) {
        getProperty(rearRightCanID, motor -> motor.getPIDController().setReference(velocity, ControlType.kSmartVelocity));
    }


}