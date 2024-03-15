package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class Intake extends SubsystemBase {
    private final VictorSPX intakeMotor;
    // private final VictorSPX bumperMotor;

    private final VictorSPX loaderMotor;


    // for brevity's sake
    private final VictorSPXControlMode PercentOutput = VictorSPXControlMode.PercentOutput;

    public Command intakeCommand() {
        return startEnd(
            () -> runIntakeMotors(true), 
            () -> runIntakeMotors(false)
        );
    }
    public Command reverseCommand() {
        return startEnd(
            () -> outTakeMotors(true), 
            () -> runIntakeMotors(false)
        );
    }
    public void outTakeMotors(boolean on) {
        if (on) {
            intakeMotor.set(PercentOutput, -0.3*SubsystemConstants.intakeMotorSpeed);
            // bumperMotor.set(PercentOutput, SubsystemConstants.bumperMotorIntakeSpeed);
        } else {
            intakeMotor.set(PercentOutput, 0);
            // bumperMotor.set(PercentOutput, SubsystemConstants.bumperMotorRejectSpeed);
        }
    }

    /**
     *  Move the loader according to the left stick, ONLY IF both controller triggers are NOT fully pressed
     */

    public Command loadCommand(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger, DoubleSupplier speed) {
        return runOnce(
            () -> {
                //make sure that we're not in Climber Mode
                if (leftTrigger.getAsDouble() != 1 ||  rightTrigger.getAsDouble() != 1)
                    runLoaderMotors(speed.getAsDouble());
                else
                    runLoaderMotors(0);
            }
            // ,
            // () -> runLoaderMotors(0)
        );
    }

    public void runIntakeMotors(boolean on) {
        if (on) {
            intakeMotor.set(PercentOutput, 0.7*SubsystemConstants.intakeMotorSpeed);
            // bumperMotor.set(PercentOutput, SubsystemConstants.bumperMotorIntakeSpeed);
        } else {
            intakeMotor.set(PercentOutput, 0);
            // bumperMotor.set(PercentOutput, SubsystemConstants.bumperMotorRejectSpeed);
        }
    }

    public void runLoaderMotors(double speed) {
        if (speed >= 0) intakeMotor.set(PercentOutput, speed * SubsystemConstants.intakeMotorSpeed);
        loaderMotor.set(PercentOutput, speed * SubsystemConstants.loaderMotorSpeed);
    }

    public Intake(int intakeMotorID, int loaderMotorID) {
        intakeMotor = new VictorSPX(intakeMotorID);
        intakeMotor.configFactoryDefault();
        intakeMotor.setInverted(true);
        // bumperMotor = new VictorSPX(bumperMotorID);
        // bumperMotor.configFactoryDefault();

        loaderMotor = new VictorSPX(loaderMotorID);
        loaderMotor.configFactoryDefault();


        // bumperMotor.set(PercentOutput, SubsystemConstants.bumperMotorRejectSpeed);
    }
    
}
