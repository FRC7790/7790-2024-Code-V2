package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    private CANSparkMax intakeMotor;
    private CANSparkMax indexMotor;

    public Shooter() {
        this.shooterMotor1 = new CANSparkMax(30, CANSparkLowLevel.MotorType.kBrushless);
        this.shooterMotor2 = new CANSparkMax(31, CANSparkLowLevel.MotorType.kBrushless);
        this.indexMotor = new CANSparkMax(32, CANSparkLowLevel.MotorType.kBrushless);
        this.intakeMotor = new CANSparkMax(33, CANSparkLowLevel.MotorType.kBrushless);
    }
    public void startShooter() {
        shooterMotor1.set(-.5);
        shooterMotor2.set(.3);
        indexMotor.set(.8);
    }
    public void stopShooter() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
        indexMotor.set(0);
    }
    public void harvest() {
        intakeMotor.set(.4);
        indexMotor.set(.2);
    }
    public void harvestStop() {
        intakeMotor.set(0);
        indexMotor.set(0);
    }
    public void harvestReverse() {
        indexMotor.set(-.2);
        intakeMotor.set(-.2);
    }
    public void shoot() {
        intakeMotor.set(.8);
    }
    public void indexStop() {
        intakeMotor.set(0);
    }
}