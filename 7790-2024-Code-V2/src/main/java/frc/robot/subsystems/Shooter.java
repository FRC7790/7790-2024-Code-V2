package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    private CANSparkMax shooterMotor3;
    private CANSparkMax shooterMotor4;
    private CANSparkMax intakeMotor;
    private CANSparkMax indexMotor;
    private RelativeEncoder shooterMotor1Encoder;
    private RelativeEncoder shooterMotor2Encoder;
    private RelativeEncoder shooterMotor3Encoder;
    private RelativeEncoder shooterMotor4Encoder;
    private RelativeEncoder intakeMotorEncoder;
    private RelativeEncoder indexMotorEncoder;
    private float desiredVelShooterMotor1;
    private float desiredVelShooterMotor3;
    private float desiredVelIntakeMotor;
    private float desiredVelIndexMotor;
    PIDController pid;

    public Shooter() {

        this.desiredVelShooterMotor1 = 0.0f;
        this.desiredVelShooterMotor3 = 0.0f;
        this.desiredVelIntakeMotor = 0.0f;
        this.desiredVelIndexMotor = 0.0f;
        // In RPM!!! ^

        this.shooterMotor1 = new CANSparkMax(30, CANSparkLowLevel.MotorType.kBrushless);
        this.shooterMotor2 = new CANSparkMax(31, CANSparkLowLevel.MotorType.kBrushless);
        this.indexMotor = new CANSparkMax(32, CANSparkLowLevel.MotorType.kBrushless);
        this.intakeMotor = new CANSparkMax(33, CANSparkLowLevel.MotorType.kBrushless);
        
        this.pid = new PIDController(0.0001, 0.0, 0.0);

        shooterMotor1Encoder = shooterMotor1.getEncoder();
        shooterMotor2Encoder = shooterMotor2.getEncoder();
        shooterMotor3Encoder = shooterMotor3.getEncoder();
        shooterMotor4Encoder = shooterMotor4.getEncoder();
        intakeMotorEncoder = intakeMotor.getEncoder();
        indexMotorEncoder = indexMotor.getEncoder();
        

        this.shooterMotor2.follow(this.shooterMotor1, true);
        this.shooterMotor4.follow(this.shooterMotor3, true);

      
    }
    public void startShooter() {
        desiredVelShooterMotor1 = 6000f;
        desiredVelShooterMotor3 = 5500f;
        desiredVelIndexMotor = 4000f;
    }
    public void stopShooter() {
        desiredVelShooterMotor1 = 0f;
        desiredVelShooterMotor3 = 0f;
        desiredVelIndexMotor = 0f;
    }
    public void harvest() {
        desiredVelIntakeMotor = 2000f;
        desiredVelIndexMotor = 2100f;
    }
    public void harvestStop() {
        desiredVelIntakeMotor = 0f;
        desiredVelIndexMotor = 0f;
    }
    public void harvestReverse() {
        desiredVelIntakeMotor = -500f;
        desiredVelIndexMotor = -500f;
    }
    public void shoot() {
        desiredVelIntakeMotor = 6000f;
    }
   /*  public void indexStop() {
        intakeMotor.set(0);
    }*/

        @Override
    public void periodic() {

        final float shooterMaxoutput = 0.5f;
        final float intakeMaxoutput = 0.5f;
        final float indexMaxoutput = 0.5f;

        final double output = MathUtil.clamp(this.pid.calculate(shooterMotor1Encoder.getVelocity(), this.desiredVelShooterMotor1), -shooterMaxoutput, shooterMaxoutput);
        shooterMotor1.set(output);

        final double output2 = MathUtil.clamp(this.pid.calculate(shooterMotor3Encoder.getVelocity(), -(this.desiredVelShooterMotor3)), -shooterMaxoutput, shooterMaxoutput);
        shooterMotor3.set(output2);

        final double output3 = MathUtil.clamp(this.pid.calculate(intakeMotorEncoder.getVelocity(), this.desiredVelIntakeMotor), -intakeMaxoutput, intakeMaxoutput);
        intakeMotor.set(output3);

        final double output4 = MathUtil.clamp(this.pid.calculate(indexMotorEncoder.getVelocity(), this.desiredVelIndexMotor), -indexMaxoutput, indexMaxoutput);
        indexMotor.set(output4);
    }
}