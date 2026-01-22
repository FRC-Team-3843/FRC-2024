// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShieldSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final ShieldSubsystem m_shieldSubsystem = new ShieldSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Auto chooser
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Start camera
    CameraServer.startAutomaticCapture();

    // Configure the trigger bindings
    configureBindings();

    // Configure autonomous chooser
    configureAutoChooser();

    // Set default commands
    setDefaultCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // A button - Intake mode
    // (pivot down, shooter/feeder reverse, shield down when pivot passes low position)
    m_driverController
        .a()
        .or(m_operatorController.a())
        .whileTrue(
            Commands.parallel(
                    m_shooterSubsystem.spinReverseCommand(),
                    m_shooterSubsystem.feedReverseCommand(),
                    m_pivotSubsystem.moveToIntakeCommand())
                .andThen(
                    Commands.waitUntil(
                        () ->
                            m_pivotSubsystem.getPosition()
                                > Constants.PivotConstants.SHOOTING_LOW_POS))
                .andThen(m_shieldSubsystem.moveDownCommand()));

    // B button (without RB) - High shot prep
    // (pivot high, shooter spin, shield mid)
    m_driverController
        .b()
        .and(m_driverController.rightBumper().negate())
        .or(
            m_operatorController
                .b()
                .and(m_operatorController.rightBumper().negate()))
        .whileTrue(
            Commands.parallel(
                m_shooterSubsystem.spinUpCommand(),
                m_pivotSubsystem.moveToHighCommand(),
                m_shieldSubsystem.moveMidCommand()));

    // B + RB - High shot fire
    // (pivot high, shooter spin, feeder on, shield mid)
    m_driverController
        .b()
        .and(m_driverController.rightBumper())
        .or(
            m_operatorController
                .b()
                .and(m_operatorController.rightBumper()))
        .whileTrue(
            Commands.parallel(
                m_shooterSubsystem.spinUpCommand(),
                m_shooterSubsystem.feedCommand(),
                m_pivotSubsystem.moveToHighCommand(),
                m_shieldSubsystem.moveMidCommand()));

    // X button (without RB) - Low shot prep
    // (pivot low, shield up)
    m_driverController
        .x()
        .and(m_driverController.rightBumper().negate())
        .or(
            m_operatorController
                .x()
                .and(m_operatorController.rightBumper().negate()))
        .whileTrue(
            Commands.parallel(
                m_pivotSubsystem.moveToLowCommand(), m_shieldSubsystem.moveUpCommand()));

    // X + RB - Low shot fire
    // (pivot low, shooter spin, feeder on, shield up)
    m_driverController
        .x()
        .and(m_driverController.rightBumper())
        .or(
            m_operatorController
                .x()
                .and(m_operatorController.rightBumper()))
        .whileTrue(
            Commands.parallel(
                m_shooterSubsystem.spinUpCommand(),
                m_shooterSubsystem.feedCommand(),
                m_pivotSubsystem.moveToLowCommand(),
                m_shieldSubsystem.moveUpCommand()));
  }

  /** Sets default commands for subsystems. */
  private void setDefaultCommands() {
    // Default drive command - mecanum drive with joysticks
    m_driveSubsystem.setDefaultCommand(
        m_driveSubsystem.driveCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    // Default stowed command - pivot high, shield mid when below low position
    m_pivotSubsystem.setDefaultCommand(
        Commands.run(
                () -> {
                  m_pivotSubsystem.moveToHigh();
                  m_shooterSubsystem.stopAll();
                  if (m_pivotSubsystem.getPosition()
                      < Constants.PivotConstants.SHOOTING_LOW_POS) {
                    m_shieldSubsystem.moveMid();
                  }
                },
                m_pivotSubsystem,
                m_shooterSubsystem,
                m_shieldSubsystem)
            .withName("Stowed"));
  }

  /** Configures the autonomous chooser with all auto modes. */
  private void configureAutoChooser() {
    m_autoChooser.setDefaultOption(Constants.AutoConstants.DO_NOTHING, doNothingAuto());
    m_autoChooser.addOption(Constants.AutoConstants.AUTO_1, shootOnlyAuto());
    m_autoChooser.addOption(Constants.AutoConstants.AUTO_2, centerDoubleAuto());
    m_autoChooser.addOption(Constants.AutoConstants.AUTO_3, rightSideDoubleAuto());
    m_autoChooser.addOption(Constants.AutoConstants.AUTO_4, leftSideDoubleAuto());
    m_autoChooser.addOption(Constants.AutoConstants.AUTO_5, rightSideMoveAuto());
    m_autoChooser.addOption(Constants.AutoConstants.AUTO_6, leftSideMoveAuto());

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  // ==================== Autonomous Commands ====================

  /** Do nothing auto - all motors off. */
  private Command doNothingAuto() {
    return Commands.run(
            () -> {
              m_driveSubsystem.stop();
              m_shooterSubsystem.stopAll();
              m_pivotSubsystem.stop();
            },
            m_driveSubsystem,
            m_shooterSubsystem,
            m_pivotSubsystem)
        .withName("Do Nothing");
  }

  /** Auto 1 - Just shoot high. */
  private Command shootOnlyAuto() {
    return Commands.sequence(
            m_driveSubsystem.resetEncodersCommand(),
            Commands.parallel(
                m_pivotSubsystem.moveToHighCommand(),
                m_shooterSubsystem.spinUpCommand(),
                m_shieldSubsystem.moveMidCommand()),
            Commands.waitSeconds(Constants.AutoConstants.SHOOTER_SPINUP_TIME),
            m_shooterSubsystem.feedCommand(),
            Commands.waitSeconds(Constants.AutoConstants.FEED_TIME),
            m_shooterSubsystem.stopCommand())
        .withName("Auto 1 - Shoot Only");
  }

  /** Auto 2 - Center field double auto. */
  private Command centerDoubleAuto() {
    return Commands.sequence(
            // First shot
            m_driveSubsystem.resetEncodersCommand(),
            Commands.parallel(
                m_pivotSubsystem.moveToHighCommand(),
                m_shooterSubsystem.spinUpCommand(),
                m_shieldSubsystem.moveMidCommand()),
            Commands.waitSeconds(Constants.AutoConstants.SHOOTER_SPINUP_TIME),
            m_shooterSubsystem.feedCommand(),
            Commands.waitSeconds(Constants.AutoConstants.FEED_TIME),
            // Move to intake
            m_shooterSubsystem.stopCommand(),
            Commands.parallel(
                m_pivotSubsystem.moveToIntakeCommand(), m_shieldSubsystem.moveDownCommand()),
            Commands.waitUntil(
                () ->
                    Math.abs(
                            m_pivotSubsystem.getPosition()
                                - Constants.PivotConstants.INTAKE_POS)
                        < Constants.PivotConstants.POSITION_TOLERANCE),
            // Drive back while intaking
            Commands.parallel(
                m_shooterSubsystem.spinReverseCommand(),
                m_shooterSubsystem.feedReverseCommand(),
                m_driveSubsystem.driveToPositionCommand(-25, -25)),
            // Return and shoot
            m_shooterSubsystem.stopCommand(),
            m_pivotSubsystem.moveToHighCommand(),
            Commands.waitUntil(
                () ->
                    m_pivotSubsystem.getPosition()
                        < Constants.PivotConstants.SHOOTING_LOW_POS),
            m_shieldSubsystem.moveMidCommand(),
            m_driveSubsystem.driveToPositionCommand(0, 0),
            m_shooterSubsystem.spinUpCommand(),
            Commands.waitSeconds(Constants.AutoConstants.SHOOTER_SPINUP_TIME),
            m_shooterSubsystem.feedCommand(),
            Commands.waitSeconds(Constants.AutoConstants.FEED_TIME),
            m_shooterSubsystem.stopCommand())
        .withName("Auto 2 - Center Double");
  }

  /** Auto 3 - Right side double auto. */
  private Command rightSideDoubleAuto() {
    return Commands.sequence(
            // First shot
            m_driveSubsystem.resetEncodersCommand(),
            Commands.parallel(
                m_pivotSubsystem.moveToHighCommand(),
                m_shooterSubsystem.spinUpCommand(),
                m_shieldSubsystem.moveMidCommand()),
            Commands.waitSeconds(Constants.AutoConstants.SHOOTER_SPINUP_TIME),
            m_shooterSubsystem.feedCommand(),
            Commands.waitSeconds(Constants.AutoConstants.FEED_TIME),
            // Move to intake
            m_shooterSubsystem.stopCommand(),
            Commands.parallel(
                m_pivotSubsystem.moveToIntakeCommand(), m_shieldSubsystem.moveDownCommand()),
            Commands.waitUntil(
                () ->
                    Math.abs(
                            m_pivotSubsystem.getPosition()
                                - Constants.PivotConstants.INTAKE_POS)
                        < Constants.PivotConstants.POSITION_TOLERANCE),
            // Drive back
            Commands.parallel(
                m_shooterSubsystem.spinReverseCommand(),
                m_shooterSubsystem.feedReverseCommand(),
                m_driveSubsystem.driveToPositionCommand(-15, -15)),
            // Turn right
            m_driveSubsystem.driveToPositionCommand(-45, -15),
            // Drive back more
            m_driveSubsystem.driveToPositionCommand(-65, -35),
            // Drive forward
            m_driveSubsystem.driveToPositionCommand(-45, -15),
            // Prepare to shoot
            Commands.parallel(
                m_shooterSubsystem.stopCommand(),
                m_pivotSubsystem.moveToHighCommand(),
                Commands.waitUntil(
                        () ->
                            m_pivotSubsystem.getPosition()
                                < Constants.PivotConstants.SHOOTING_LOW_POS)
                    .andThen(m_shieldSubsystem.moveMidCommand())),
            // Turn left
            m_driveSubsystem.driveToPositionCommand(-15, -15),
            // Drive forward
            m_driveSubsystem.driveToPositionCommand(0, 0),
            // Shoot
            m_shooterSubsystem.spinUpCommand(),
            Commands.waitSeconds(Constants.AutoConstants.SHOOTER_SPINUP_TIME),
            m_shooterSubsystem.feedCommand(),
            Commands.waitSeconds(Constants.AutoConstants.FEED_TIME),
            m_shooterSubsystem.stopCommand())
        .withName("Auto 3 - Right Side Double");
  }

  /** Auto 4 - Left side double auto. */
  private Command leftSideDoubleAuto() {
    return Commands.sequence(
            // First shot
            m_driveSubsystem.resetEncodersCommand(),
            Commands.parallel(
                m_pivotSubsystem.moveToHighCommand(),
                m_shooterSubsystem.spinUpCommand(),
                m_shieldSubsystem.moveMidCommand()),
            Commands.waitSeconds(Constants.AutoConstants.SHOOTER_SPINUP_TIME),
            m_shooterSubsystem.feedCommand(),
            Commands.waitSeconds(Constants.AutoConstants.FEED_TIME),
            // Move to intake
            m_shooterSubsystem.stopCommand(),
            Commands.parallel(
                m_pivotSubsystem.moveToIntakeCommand(), m_shieldSubsystem.moveDownCommand()),
            Commands.waitUntil(
                () ->
                    Math.abs(
                            m_pivotSubsystem.getPosition()
                                - Constants.PivotConstants.INTAKE_POS)
                        < Constants.PivotConstants.POSITION_TOLERANCE),
            // Drive back
            Commands.parallel(
                m_shooterSubsystem.spinReverseCommand(),
                m_shooterSubsystem.feedReverseCommand(),
                m_driveSubsystem.driveToPositionCommand(-15, -15)),
            // Turn left
            m_driveSubsystem.driveToPositionCommand(-15, -45),
            // Drive back more
            m_driveSubsystem.driveToPositionCommand(-35, -65),
            // Drive forward
            m_driveSubsystem.driveToPositionCommand(-15, -45),
            // Prepare to shoot
            Commands.parallel(
                m_shooterSubsystem.stopCommand(),
                m_pivotSubsystem.moveToHighCommand(),
                Commands.waitUntil(
                        () ->
                            m_pivotSubsystem.getPosition()
                                < Constants.PivotConstants.SHOOTING_LOW_POS)
                    .andThen(m_shieldSubsystem.moveMidCommand())),
            // Turn right
            m_driveSubsystem.driveToPositionCommand(-15, -15),
            // Drive forward
            m_driveSubsystem.driveToPositionCommand(0, 0),
            // Shoot
            m_shooterSubsystem.spinUpCommand(),
            Commands.waitSeconds(Constants.AutoConstants.SHOOTER_SPINUP_TIME),
            m_shooterSubsystem.feedCommand(),
            Commands.waitSeconds(Constants.AutoConstants.FEED_TIME),
            m_shooterSubsystem.stopCommand())
        .withName("Auto 4 - Left Side Double");
  }

  /** Auto 5 - Right side move auto. */
  private Command rightSideMoveAuto() {
    return Commands.sequence(
            // Shoot
            m_driveSubsystem.resetEncodersCommand(),
            Commands.parallel(
                m_pivotSubsystem.moveToHighCommand(),
                m_shooterSubsystem.spinUpCommand(),
                m_shieldSubsystem.moveMidCommand()),
            Commands.waitSeconds(Constants.AutoConstants.SHOOTER_SPINUP_TIME),
            m_shooterSubsystem.feedCommand(),
            Commands.waitSeconds(Constants.AutoConstants.FEED_TIME),
            // Stop and wait
            m_shooterSubsystem.stopCommand(),
            Commands.waitSeconds(Constants.AutoConstants.WAIT_TIME),
            // Move
            m_driveSubsystem.driveToPositionCommand(-47, -15),
            m_driveSubsystem.driveToPositionCommand(-62, -30))
        .withName("Auto 5 - Right Side Move");
  }

  /** Auto 6 - Left side move auto. */
  private Command leftSideMoveAuto() {
    return Commands.sequence(
            // Shoot
            m_driveSubsystem.resetEncodersCommand(),
            Commands.parallel(
                m_pivotSubsystem.moveToHighCommand(),
                m_shooterSubsystem.spinUpCommand(),
                m_shieldSubsystem.moveMidCommand()),
            Commands.waitSeconds(Constants.AutoConstants.SHOOTER_SPINUP_TIME),
            m_shooterSubsystem.feedCommand(),
            Commands.waitSeconds(Constants.AutoConstants.FEED_TIME),
            // Stop and wait
            m_shooterSubsystem.stopCommand(),
            Commands.waitSeconds(Constants.AutoConstants.WAIT_TIME),
            // Move
            m_driveSubsystem.driveToPositionCommand(-15, -47),
            m_driveSubsystem.driveToPositionCommand(-30, -62))
        .withName("Auto 6 - Left Side Move");
  }
}
