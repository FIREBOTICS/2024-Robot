package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class Intake extends SubsystemBase {
    private final VictorSPX intakeMotor;
    private final VictorSPX bumperMotor;

    private final VictorSPX leftLoaderMotor;
    private final VictorSPX rightLoaderMotor;


    // for brevity's sake
    private final VictorSPXControlMode PercentOutput = VictorSPXControlMode.PercentOutput;

    public void runIntakeMotors(boolean on) {
        if (on) {
            intakeMotor.set(PercentOutput, SubsystemConstants.intakeMotorSpeed);
            bumperMotor.set(PercentOutput, SubsystemConstants.bumperMotorIntakeSpeed);
        } else {
            intakeMotor.set(PercentOutput, 0);
            bumperMotor.set(PercentOutput, SubsystemConstants.bumperMotorRejectSpeed);
        }
    }

    public void runLoaderMotors(double speed) {
        intakeMotor.set(PercentOutput, SubsystemConstants.intakeMotorSpeed);
        leftLoaderMotor.set(PercentOutput, speed);
        rightLoaderMotor.set(PercentOutput, speed);
    }

    public Intake(int intakeMotorID, int bumperMotorID, int loaderLeftID, int loaderRightId) {
        intakeMotor = new VictorSPX(intakeMotorID);
        bumperMotor = new VictorSPX(bumperMotorID);
        intakeMotor.configFactoryDefault();
        bumperMotor.configFactoryDefault();

        leftLoaderMotor =  new VictorSPX(loaderLeftID);
        rightLoaderMotor = new VictorSPX(loaderRightId);
        leftLoaderMotor .configFactoryDefault();
        rightLoaderMotor.configFactoryDefault();


        bumperMotor.set(PercentOutput, SubsystemConstants.bumperMotorRejectSpeed);
    }
    
}
