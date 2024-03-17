// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.intake.SetCommand;
import frc.robot.commands.shooter.AmpCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Initialize subsystems.

  /**
   * <h1>THESE VALUES ARE SERIOUSLY WRONG!!!!!</h1>
   * 
   * THE ROBOT SUFFERED SERIOUS DAMAGE AND WE HAD
   * TO SWITCH LOTS OF MOTOR IDS AND ENCODER IDS.
   * 
   * PLEASE PLEASE PLEASE PLEASE PLEASE CHANGE
   * THESE VALUES <b>BEFOREEEEE</b> RUNNING IT!!!!
   */

  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final Intake m_intake = new Intake(50, 51);
  private final Shooter m_shooter = new Shooter(52, 53, 8, 10, 9, 11);
  private final Climber m_climber = new Climber(54, 55);
  // private final LEDs m_leds = new LEDs(0,100);
  // UsbCamera camera = CameraServer.startAutomaticCapture();

  // Initialize auto selector.
  SendableChooser<Command> autoSelector = new SendableChooser<Command>();
  private final CommandXboxController m_driverController = new CommandXboxController(
      ControllerConstants.driverControllerPort);

  private final CommandXboxController m_codriverController = new CommandXboxController(
      ControllerConstants.codriverControllerPort);

  private boolean isFieldOriented = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    Field2d field = new Field2d();

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      field.setRobotPose(pose);
    });

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });

    SmartDashboard.putData("Field", field);

    NamedCommands.registerCommand("Shoot Speaker", m_shooter.shootCommand(SubsystemConstants.speakerShotSpeed));
    NamedCommands.registerCommand("Shoot Amp", new AmpCommand(m_shooter));
    NamedCommands.registerCommand("Shooter 0", m_shooter.shootCommand(0));
    NamedCommands.registerCommand("Intake", m_intake.loadCommand(() -> 0, () -> 0, () -> 1));
    NamedCommands.registerCommand("Intake 0", m_intake.loadCommand(() -> 0, () -> 0, () -> 0));
    NamedCommands.registerCommand("Nudge Note", new SetCommand(m_intake, m_shooter));

    autoSelector.addOption("Amp 2 Speaker 1", new PathPlannerAuto("Amp 2 Speaker 1"));
    autoSelector.addOption("Amp 3", new PathPlannerAuto("Amp 3"));
    autoSelector.addOption("Speaker 2", new PathPlannerAuto("Speaker 2"));
    autoSelector.addOption("Test Curve", new PathPlannerAuto("Test Curve"));
    autoSelector.addOption("Only Speaker", new PathPlannerAuto("Only Speaker"));
    SmartDashboard.putData("auto selector", autoSelector);

    // camera.setResolution(240, 190);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger (java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public void configureBindings() {
    /*
     * // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
     * new Trigger(m_exampleSubsystem::exampleCondition)
     * .onTrue(new ExampleCommand(m_exampleSubsystem));
     */

    /*
     * Y
     * X B
     * A
     */
    // Schedule `lock` when the Xbox controller's left trigger is beyond the
    // threshold,
    // cancelling on release.
    // m_driverController.leftTrigger(ControllerConstants.triggerPressedThreshhold).whileTrue(m_swerveDrive.lockCommand());
    m_driverController.y().onTrue(m_swerveDrive.speedUpCommand(0.1));
    m_driverController.a().onTrue(m_swerveDrive.slowDownCommand(0.1));
    m_driverController.x().onTrue(m_shooter.toggleCompressor());
    m_driverController.back().onTrue(toggleFieldOrientedCommand());
    m_driverController.start().onTrue(m_swerveDrive.resetHeadingCommand());
    m_driverController.povLeft() /* GOTO STAGE POSITION #1 */; // unclear if these will be feasible
    m_driverController.povLeft() /* GOTO STAGE POSITION #2 */;
    m_driverController.povLeft() /* GOTO STAGE POSITION #3 */;

    // Extend AMP platform doohickey when either bumper is pressed, and lower it
    // when it's unpressed
    /* */
    // m_codriverController.leftBumper().or(m_codriverController.rightBumper())
    // .whileTrue(m_shooter.extendPlatformCommand());
    // .onTrue(m_shooter.setPlatformCommand(true));
    // .onFalse(m_shooter.setPlatformCommand(false));
    // m_codriverController.x().whileTrue(new IntakeCommand(m_intake, m_leds));
    m_codriverController.b().whileTrue(new AmpCommand(m_shooter));
    m_codriverController.a().whileTrue(m_shooter.shootCommand(SubsystemConstants.speakerShotSpeed));
    m_codriverController.x().whileTrue(new SetCommand(m_intake, m_shooter));
    m_codriverController.y().whileTrue(m_intake.reverseCommand());

    m_codriverController.leftBumper().or(m_codriverController.rightBumper())
        .onTrue(Commands.waitSeconds(1).deadlineWith(m_shooter.shootCommand(-0.5)));
    /* Uncomment this if the other one doesn't work */
    // m_codriverController.b().onTrue(m_shooter.shootCommand(SubsystemConstants.ampShotSpeed))
    // .onFalse(m_shooter.shootCommand(0));

    m_swerveDrive.setDefaultCommand(
        m_swerveDrive.driveCommand(
            () -> deadband(m_driverController.getLeftY()),
            () -> deadband(m_driverController.getLeftX()),
            () -> deadband(m_driverController.getRightX()),
            () -> isFieldOriented // Switch to False for Chairbot Mode
        ));

    // m_intake.setDefaultCommand(
    // new LoadCommand(
    // () -> deadband(m_codriverController.getLeftY()),
    // m_intake,
    // m_leds)
    // );

    m_intake.setDefaultCommand(
        m_intake.loadCommand(
            () -> m_codriverController.getLeftTriggerAxis(),
            () -> m_codriverController.getRightTriggerAxis(),
            () -> -deadband(m_codriverController.getLeftY())));

    m_shooter.setDefaultCommand(
        m_shooter.shootCommand(
            () -> -0.5 * deadband(m_codriverController.getRightY())));

    m_climber.setDefaultCommand(
        m_climber.moveClimber(
            () -> m_codriverController.getLeftTriggerAxis(),
            () -> m_codriverController.getRightTriggerAxis(),
            () -> m_codriverController.getLeftY(),
            () -> m_codriverController.getRightY()));

    /*
     * m_leds.setDefaultCommand(
     * m_leds.defaultColorCommand()
     * );
     */
  }

  /**
   * Deadbands inputs to eliminate tiny unwanted values from the joysticks or
   * gamepad sticks.
   * <p>
   * If the distance between the input and zero is less than the deadband amount,
   * the output will be zero.
   * Otherwise, the value will not change.
   * 
   * @param value The controller value to deadband.
   * @return The deadbanded controller value.
   */
  public double deadband(double value) {
    if (Math.abs(value) < ControllerConstants.joystickDeadband)
      return 0.0;
    return value;
  }

  public boolean getFieldOriented() {
    return isFieldOriented;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  /**
   * Use this to pass the disabled command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getdisabledCommand() {
    // return new PathPlannerAuto("AMP 2 Shoot 1 - Auto 1.1");
    // return Commands.runOnce()
    return null;
  }

  public Command toggleFieldOrientedCommand() {
    return Commands.runOnce(
        () -> isFieldOriented = !isFieldOriented);
  }
}
