package frc.robot.subsystems;

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
    private final CANSparkMax leftLoaderMotor;
    private final CANSparkMax rightLoaderMotor;

    private final CANSparkMax leftShooterMotor;
    private final CANSparkMax rightShooterMotor;

    private final Solenoid leftSolenoid;
    private final Solenoid rightSolenoid;

    public Command setPlatformCommand(boolean status) {
        return runOnce(
            () -> {
                leftSolenoid.set(status);
                rightSolenoid.set(status);
            });
    }
    public Command shootCommand(double speed) {
        return startEnd(
            () -> {
                leftShooterMotor.set(speed);
                rightShooterMotor.set(speed);
            },
            () -> {
                leftShooterMotor.set(0);
                rightShooterMotor.set(0);

            });
    }

    public Shooter(int loaderLeftID, int loaderRightId, int shooterLeftId, int shooterRightId, int leftSolenoidId, int rightSolenoidId) {
        leftLoaderMotor = new CANSparkMax(loaderLeftID, MotorType.kBrushless);
        rightLoaderMotor = new CANSparkMax(loaderRightId, MotorType.kBrushless);
        leftLoaderMotor.restoreFactoryDefaults();
        rightLoaderMotor.restoreFactoryDefaults();

        leftShooterMotor = new CANSparkMax(shooterLeftId, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(shooterRightId, MotorType.kBrushless);
        leftShooterMotor.restoreFactoryDefaults();
        rightShooterMotor.restoreFactoryDefaults();

        leftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, leftSolenoidId);
        rightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, rightSolenoidId);
    }
}
