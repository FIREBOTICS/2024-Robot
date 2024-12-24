package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Shooter;

public class AmpCommand extends Command {
    private final Shooter shooter;

    public AmpCommand(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.runMotors(SubsystemConstants.ampShotSpeed);
        shooter.setPistons(DoubleSolenoid.Value.kForward);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.runMotors(0);
        shooter.setPistons(DoubleSolenoid.Value.kReverse);
    }
}
