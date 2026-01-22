package frc.robot.subsystems;

/**
 * DriveIO is the "Universal Remote" interface for our drivetrain.
 * 
 * <p>It lists all the actions we might want to do (like "Set Voltage" or "Get Speed"),
 * but it doesn't say HOW to do them. The specific instructions for each motor type
 * are in the implementation files (DriveIOSparkMax.java and DriveIOTalon.java).
 * 
 * <p>By using this, the main robot code (DriveSubsystem) doesn't need to know
 * if we are using NEOs or CIMs. It just pushes the buttons on this remote.
 */
public interface DriveIO {
  /**
   * Applies electrical power (voltage) directly to the motors.
   * This is "Open Loop" control - we just give power and hope it moves.
   *
   * @param frontLeftVolts Voltage for front left motor (-12.0 to 12.0)
   * @param rearLeftVolts Voltage for rear left motor (-12.0 to 12.0)
   * @param frontRightVolts Voltage for front right motor (-12.0 to 12.0)
   * @param rearRightVolts Voltage for rear right motor (-12.0 to 12.0)
   */
  void setVoltages(double frontLeftVolts, double rearLeftVolts, double frontRightVolts, double rearRightVolts);

  /**
   * Tells the motors to spin at a specific speed (RPM).
   * This is "Closed Loop" control - the motor checks its sensor and adjusts power to match the target.
   * 
   * <p>NOTE: If we don't have sensors (Encoders), this might just guess the voltage needed.
   *
   * @param frontLeftVel Target velocity (RPM)
   * @param rearLeftVel Target velocity (RPM)
   * @param frontRightVel Target velocity (RPM)
   * @param rearRightVel Target velocity (RPM)
   */
  void setVelocities(double frontLeftVel, double rearLeftVel, double frontRightVel, double rearRightVel);

  /**
   * Tells the motors to drive to a specific total rotation count.
   * Used for autonomous movement (e.g., "Drive forward 10 wheel rotations").
   *
   * @param leftPos Target position for left side (Rotations)
   * @param rightPos Target position for right side (Rotations)
   */
  void setPositionTargets(double leftPos, double rightPos);

  /**
   * Asks the motors: "Where are you right now?"
   *
   * @return Array of positions {FrontLeft, RearLeft, FrontRight, RearRight} in Rotations
   */
  double[] getPositions();

  /**
   * Asks the motors: "How fast are you spinning right now?"
   *
   * @return Array of velocities {FrontLeft, RearLeft, FrontRight, RearRight} in RPM
   */
  double[] getVelocities();

  /** 
   * Resets the "Zero" point for the sensors. 
   * Usually done at the start of a match or auto routine.
   */
  void resetEncoders();

  /** 
   * Emergency Stop. Cuts power to all motors. 
   */
  void stop();
}