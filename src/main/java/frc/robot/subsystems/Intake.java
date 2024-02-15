package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class Intake extends SubsystemBase {
    private final VictorSPX intakeMotor;
    private final VictorSPX bumperMotor;

    // for brevity's sake
    private final VictorSPXControlMode PercentOutput = VictorSPXControlMode.PercentOutput;

    public Command intakeCommand() {
        return startEnd(
            () -> {
                intakeMotor.set(PercentOutput, SubsystemConstants.intakeMotorSpeed);
                bumperMotor.set(PercentOutput, SubsystemConstants.bumperMotorIntakeSpeed);
            },
            () -> {
                intakeMotor.set(PercentOutput, 0);
                bumperMotor.set(PercentOutput, SubsystemConstants.bumperMotorRejectSpeed);
            });
    }

    public Intake(int intakeMotorID, int bumperMotorID) {
        intakeMotor = new VictorSPX(intakeMotorID);
        bumperMotor = new VictorSPX(bumperMotorID);
        intakeMotor.configFactoryDefault();
        bumperMotor.configFactoryDefault();

        bumperMotor.set(PercentOutput, SubsystemConstants.bumperMotorRejectSpeed);
    }
    
}
