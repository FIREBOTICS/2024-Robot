package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class defines the shooter as well as the adjacent motors between the intake and shooter
 */
public class Shooter extends SubsystemBase {
    private final VictorSPX leftShooterMotor;
    private final VictorSPX rightShooterMotor;

    private final DoubleSolenoid leftSolenoid;
    private final DoubleSolenoid rightSolenoid;

    // for brevity's sake
    private final VictorSPXControlMode PercentOutput = VictorSPXControlMode.PercentOutput;

    public Command setPlatformCommand(boolean extended) {
        final DoubleSolenoid.Value value = extended ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
        return runOnce(
            () -> {
                leftSolenoid .set(value);
                rightSolenoid.set(value);
            });
    }
    public Command shootCommand(double speed) {
        return startEnd(
            () -> {
                leftShooterMotor.set(PercentOutput, speed);
                rightShooterMotor.set(PercentOutput, speed);
            },
            () -> {
                leftShooterMotor.set(PercentOutput, 0);
                rightShooterMotor.set(PercentOutput, 0);
            }); 
    }

    public void runMotors(double speed) {
        leftShooterMotor.set(PercentOutput, speed);
        rightShooterMotor.set(PercentOutput, speed);
    }

    public Shooter(int shooterLeftId, int shooterRightId, int leftSolenoidForward, int leftSolenoidReverse, int rightSolenoidForward, int rightSolenoidReverse) {
        leftShooterMotor =  new VictorSPX(shooterLeftId);
        rightShooterMotor = new VictorSPX(shooterRightId);
        leftShooterMotor .configFactoryDefault();
        rightShooterMotor.configFactoryDefault();

        leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, leftSolenoidForward, leftSolenoidReverse);
        rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, rightSolenoidForward, rightSolenoidReverse);
    }
}
