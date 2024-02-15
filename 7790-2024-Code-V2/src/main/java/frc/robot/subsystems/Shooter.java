package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        shooterMotor1.set(-.7);
        shooterMotor2.set(.5);
    }
    public void stopShooter() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }
    public void harvest() {
        intakeMotor.set(.3);
        indexMotor.set(.3);
    }
    public void harvestStop() {
        intakeMotor.set(0);
        indexMotor.set(0);
    }
    public void harvestReverse() {
        indexMotor.set(-.1);
    }
}