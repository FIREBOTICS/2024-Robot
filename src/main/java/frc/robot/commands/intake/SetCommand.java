package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SetCommand extends Command {
    private final Intake intake;
    private final Shooter shooter;

    public SetCommand(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
        intake.runLoaderMotors(-0.4);
        shooter.runMotors(-0.4);
    }

    @Override
    public void end(boolean interrupted) {
        intake.runLoaderMotors(0);
        shooter.runMotors(0);
    }
}
