package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class LoadCommand extends Command {
    private final DoubleSupplier speed;
    private final Intake intake;
    private final LEDs leds;
    private final AddressableLEDBuffer buffer;

    public LoadCommand(DoubleSupplier speed, Intake intake, LEDs leds) {
        this.speed = speed;
        this.intake = intake;
        this.leds = leds;
        this.buffer = new AddressableLEDBuffer(leds.getLength());
        for (int i = 0; i < leds.getLength(); i++) buffer.setRGB(i, 0, 255, 0);

        addRequirements(intake);
        addRequirements(leds);
    }

    @Override
    public void initialize() {
        leds.setColor(buffer);
    }

    @Override
    public void execute() {
        intake.runLoaderMotors(speed.getAsDouble()); /* runs both */
    }

    @Override
    public void end(boolean interrupted) {
        intake.runLoaderMotors(0);
        /* leds SHOULD go back to default color (defaultCommand in RobotContainer) */
    }

}
