package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class SpeakerCommand extends Command {
    // private final double speed;
    private final Shooter shooter;
    private final LEDs leds;
    private final AddressableLEDBuffer buffer;

    public SpeakerCommand(double speed, Shooter shooter, LEDs leds) {
        this.shooter = shooter;
        this.leds = leds;
        this.buffer = new AddressableLEDBuffer(leds.getLength());
        for (int i = 0; i < leds.getLength(); i++)
            buffer.setRGB(i, 255, 217, 0); /* yellow */

        addRequirements(shooter, leds);
    }

    @Override
    public void initialize() {
        shooter.runMotors(SubsystemConstants.speakerShotSpeed);
        leds.setColor(buffer);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.runMotors(0);
        /* leds SHOULD go back to default color (defaultCommand in RobotContainer) -- untested */
    }

}
