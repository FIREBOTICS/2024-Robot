package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class IntakeCommand extends Command {
    private final Intake intake;
    private final LEDs leds;
    private final AddressableLEDBuffer buffer;

    public IntakeCommand(Intake intake, LEDs leds) {
        this.intake = intake;
        this.leds = leds;
        this.buffer = new AddressableLEDBuffer(leds.getLength());
        for (int i = 0; i < leds.getLength(); i++) buffer.setRGB(i, 0, 255, 0);

        addRequirements(intake);
        addRequirements(leds);
    }

    @Override
    public void initialize() {
        intake.runIntakeMotors(true); /* runs both */
        leds.setColor(buffer);
    }

    @Override
    public void end(boolean interrupted) {
        intake.runIntakeMotors(false);
        /* leds SHOULD go back to default color (defaultCommand in RobotContainer) */
    }

}
