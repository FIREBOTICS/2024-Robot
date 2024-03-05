package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer defaultBuffer;

    private final int length;

    public Command defaultColorCommand() {
        return runOnce(
            () -> {
                ledStrip.setData(defaultBuffer);
            }
        );
    }

    public void setColor(AddressableLEDBuffer buffer) {
        ledStrip.setData(buffer);
        ledStrip.start();
    }

    public LEDs(int port, int length) {
        this.length = length;

        ledStrip = new AddressableLED(port);
        ledStrip.setLength(new AddressableLEDBuffer(length).getLength());

        defaultBuffer = new AddressableLEDBuffer(length);
        switch (DriverStation.getAlliance().get()) {
            case Red:
                for (var i = 0; i < length; i++) defaultBuffer.setRGB(i, 255, 0, 0);
                break;        
            case Blue:
                for (var i = 0; i < length; i++) defaultBuffer.setRGB(i, 0, 0, 255);
                break;
        }
    }
    
    public int getLength() {
        return length;
    }
}
