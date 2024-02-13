// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Initialize subsystems.
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final Shooter shooter = new Shooter(50, 51, 52, 53, 54, 55);
  UsbCamera camera = CameraServer.startAutomaticCapture();

  // Initialize auto selector.
  SendableChooser<Command> autoSelector = new SendableChooser<Command>();

  private final CommandXboxController m_driverController =
    new CommandXboxController(ControllerConstants.driverControllerPort);

  private final CommandXboxController m_codriverController =
    new CommandXboxController(ControllerConstants.codriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData("auto selector", autoSelector);

    camera.setResolution(240, 190);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger (java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
   private void configureBindings() {
    /*
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    */

    // Schedule `lock` when the Xbox controller's left trigger is beyond the threshold,
    // cancelling on release.
    m_driverController.leftTrigger(ControllerConstants.triggerPressedThreshhold).whileTrue(swerveDrive.lockCommand());
    m_driverController.x().onTrue(swerveDrive.speedUpCommand(0.1));
    m_driverController.a().onTrue(swerveDrive.slowDownCommand(0.1));
    m_driverController.povLeft() /* GOTO STAGE POSITION #1 */; //unclear if these will be feasible
    m_driverController.povLeft() /* GOTO STAGE POSITION #2 */;
    m_driverController.povLeft() /* GOTO STAGE POSITION #3 */;

    // Extend AMP platform doohickey when either bumper is pressed, and lower it when it's unpressed
    m_codriverController.leftBumper().or(m_codriverController.rightBumper())
    .onTrue(shooter.setPlatformCommand(true))
    .onFalse(shooter.setPlatformCommand(false));
    m_codriverController.b().whileTrue(shooter.shootCommand(SubsystemConstants.ampShotSpeed)); /* AMP */
    m_codriverController.b().whileTrue(shooter.shootCommand(SubsystemConstants.trapShotSpeed)); /* TRAP */
    m_codriverController.b().whileTrue(shooter.shootCommand(SubsystemConstants.speakerShotSpeed)); /* SPEAKER */
    // m_codriverController.b().onTrue(shooter.shootCommand(SubsystemConstants.ampShotSpeed)).onFalse(shooter.shootCommand(0));


    swerveDrive.setDefaultCommand(
        swerveDrive.driveCommand(
          () -> deadband(m_driverController.getLeftY()),
          () -> deadband(m_driverController.getLeftX()),
          () -> deadband(m_driverController.getRightX()),
          true //Switch to False for Chairbot Mode
        )
    );
  }

  /**
   * Deadbands inputs to eliminate tiny unwanted values from the joysticks or gamepad sticks.
   * <p>If the distance between the input and zero is less than the deadband amount, the output will be zero.
   * Otherwise, the value will not change.
   * 
   * @param input The controller value to deadband.
   * have different deadbands.
   * @return The deadbanded controller value.
   */
  public double deadband(double value) {
    if (Math.abs(value) < ControllerConstants.joystickDeadband)
        return 0.0;
    return value;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {
      
    };
  }
}
