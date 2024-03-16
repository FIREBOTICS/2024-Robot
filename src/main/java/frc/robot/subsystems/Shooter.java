package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final Compressor compressor;

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

    public Command extendPlatformCommand() {
        return startEnd(
            () -> {
                leftSolenoid.set(DoubleSolenoid.Value.kForward);
                rightSolenoid.set(DoubleSolenoid.Value.kForward);
            }, 
            () -> {
                leftSolenoid.set(DoubleSolenoid.Value.kReverse);
                rightSolenoid.set(DoubleSolenoid.Value.kReverse);
            }
        );
    }

    public Command shootCommand(double speed) {
        return startEnd(
            () -> {
                runMotors(speed);
            },
            () -> {
                runMotors(0);
            });
    }
    public Command shootCommand(DoubleSupplier speed) {
        return runOnce(
            () -> {
                runMotors(speed.getAsDouble());
            }
        );
    }

    public void runMotors(double speed) {
        leftShooterMotor.set(PercentOutput, speed);
        rightShooterMotor.set(PercentOutput, speed);
    }

    public void setPistons(DoubleSolenoid.Value value) {
        leftSolenoid.set(value);
        rightSolenoid.set(value);
    }

    public Command toggleCompressor() {
        return runOnce(
            () -> {
                if (compressor.isEnabled()) compressor.disable();
                else                        compressor.enableDigital();
            }
        );
    }

    public Shooter(int shooterLeftID, int shooterRightID, int leftSolenoidForward, int leftSolenoidReverse, int rightSolenoidForward, int rightSolenoidReverse) {
        leftShooterMotor =  new VictorSPX(shooterLeftID);
        rightShooterMotor = new VictorSPX(shooterRightID);
        leftShooterMotor .configFactoryDefault();
        rightShooterMotor.configFactoryDefault();
        rightShooterMotor.setInverted(true);        

        compressor = new Compressor(PneumaticsModuleType.REVPH);

        leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, leftSolenoidForward, leftSolenoidReverse);
        rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, rightSolenoidForward, rightSolenoidReverse);
        setPistons(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Compressor", compressor.isEnabled());
    }

}
