package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

/**
 * This class defines the shooter as well as the adjacent motors between the intake and shooter
 */
public class Shooter extends SubsystemBase {
    private final VictorSPX leftLoaderMotor;
    private final VictorSPX rightLoaderMotor;

    private final VictorSPX leftShooterMotor;
    private final VictorSPX rightShooterMotor;

    private final Solenoid leftSolenoid;
    private final Solenoid rightSolenoid;

    // for brevity's sake
    private final VictorSPXControlMode PercentOutput = VictorSPXControlMode.PercentOutput;

    public Command setPlatformCommand(boolean status) {
        return runOnce(
            () -> {
                leftSolenoid .set(status);
                rightSolenoid.set(status);
            });
    }
    public Command shootCommand(double speed) {
        return startEnd(
            () -> {
                leftShooterMotor .set(PercentOutput, speed);
                rightShooterMotor.set(PercentOutput, speed);
            },
            () -> {
                leftShooterMotor .set(PercentOutput, 0);
                rightShooterMotor.set(PercentOutput, 0);
            }); 
    }

    public Shooter(int loaderLeftID, int loaderRightId, int shooterLeftId, int shooterRightId, int leftSolenoidId, int rightSolenoidId) {
        leftLoaderMotor =  new VictorSPX(loaderLeftID);
        rightLoaderMotor = new VictorSPX(loaderRightId);
        leftLoaderMotor .configFactoryDefault();
        rightLoaderMotor.configFactoryDefault();

        leftShooterMotor =  new VictorSPX(shooterLeftId);
        rightShooterMotor = new VictorSPX(shooterRightId);
        leftShooterMotor .configFactoryDefault();
        rightShooterMotor.configFactoryDefault();

        leftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, leftSolenoidId);
        rightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, rightSolenoidId);
    }
}
