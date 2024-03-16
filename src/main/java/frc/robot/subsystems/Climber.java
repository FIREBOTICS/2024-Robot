package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class Climber extends SubsystemBase {
    private final VictorSPX leftMotor;
    private final VictorSPX rightMotor;

    // for brevity's sake
    private final VictorSPXControlMode PercentOutput = VictorSPXControlMode.PercentOutput;

    


    public Command moveClimber(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
        return runOnce(
            () -> {
                if (leftTrigger.getAsDouble() == 1 && rightTrigger.getAsDouble() == 1) {
                    leftMotor.set(PercentOutput,  leftSpeed.getAsDouble() * SubsystemConstants.climberMotorSpeed);
                    rightMotor.set(PercentOutput, rightSpeed.getAsDouble() * SubsystemConstants.climberMotorSpeed);
                } else {
                    leftMotor.set(PercentOutput, 0);
                    rightMotor.set(PercentOutput, 0);    
                }
            }
        );
    }

    // @Override
    // public void periodic() {
    //     climberMotor. 
    // }

    public Climber(int leftClimberID, int rightClimberID) {
        leftMotor = new VictorSPX(leftClimberID);
        rightMotor = new VictorSPX(rightClimberID);
    }
}
