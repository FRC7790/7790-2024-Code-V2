package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    private CANSparkMax shooterMotor3;
    private CANSparkMax shooterMotor4;
    private CANSparkMax intakeMotor;
    private CANSparkMax indexMotor;

    private Boolean isTriggered;
    DigitalInput noteSensor = new DigitalInput(9);

    public Shooter() {

        this.shooterMotor1 = new CANSparkMax(30, CANSparkLowLevel.MotorType.kBrushless);
        this.shooterMotor2 = new CANSparkMax(31, CANSparkLowLevel.MotorType.kBrushless);
        this.shooterMotor3 = new CANSparkMax(32, CANSparkLowLevel.MotorType.kBrushless);
        this.shooterMotor4 = new CANSparkMax(33, CANSparkLowLevel.MotorType.kBrushless);
        this.indexMotor = new CANSparkMax(35, CANSparkLowLevel.MotorType.kBrushless);
        this.intakeMotor = new CANSparkMax(34, CANSparkLowLevel.MotorType.kBrushless);
        
        this.shooterMotor1.restoreFactoryDefaults();
        this.shooterMotor2.restoreFactoryDefaults();
        this.shooterMotor3.restoreFactoryDefaults();
        this.shooterMotor4.restoreFactoryDefaults();
        this.indexMotor.restoreFactoryDefaults();
        this.intakeMotor.restoreFactoryDefaults();

        this.shooterMotor1.setIdleMode(IdleMode.kBrake);
        this.shooterMotor2.setIdleMode(IdleMode.kBrake);
        this.shooterMotor3.setIdleMode(IdleMode.kBrake);
        this.shooterMotor4.setIdleMode(IdleMode.kBrake);
        this.indexMotor.setIdleMode(IdleMode.kCoast);
        this.intakeMotor.setIdleMode(IdleMode.kCoast);
        

        this.shooterMotor2.follow(this.shooterMotor1, true);
        this.shooterMotor4.follow(this.shooterMotor3, true);
        this.isTriggered = false;
      
    }
    public void startShooter() {
        shooterMotor1.set(-.6);
        shooterMotor3.set(.4);
    }
    public void stopShooter() {
        shooterMotor1.set(0);
        shooterMotor3.set(0);
    }
    public void harvest() {
        if(isTriggered == true) {

        intakeMotor.set(.2);
        indexMotor.set(.2);
        } else {
        intakeMotor.set(0);
        indexMotor.set(0);
        }
        
    }
    public void harvestStop() {
        intakeMotor.set(0);
        indexMotor.set(0);
    }
    public void harvestReverse() {
        intakeMotor.set(-.1);
        indexMotor.set(-.1);
    }
    public void shoot() {
        indexMotor.set(.3);
    }
      public void indexStop() {
        indexMotor.set(0);
      }
    
    @Override
    public void periodic() {

        isTriggered = noteSensor.get();
    }
    }