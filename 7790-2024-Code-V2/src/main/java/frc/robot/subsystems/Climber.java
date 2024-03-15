 package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{
    private CANSparkMax climberMotor1;
    private CANSparkMax climberMotor2;
    private float desiredSpeed1 = 0;
    private float desiredSpeed2 = 0;
    private float maxSpeed = 1;


    
    public Climber() {
        this.climberMotor1 = new CANSparkMax(24, CANSparkLowLevel.MotorType.kBrushless);
        this.climberMotor2 = new CANSparkMax(25, CANSparkLowLevel.MotorType.kBrushless);

        this.climberMotor1.restoreFactoryDefaults();
        this.climberMotor1.setIdleMode(IdleMode.kBrake);

        this.climberMotor2.restoreFactoryDefaults();
        this.climberMotor2.setIdleMode(IdleMode.kBrake);

        this.climberMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10000); 
        this.climberMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10000); 

        this.climberMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000); 
        this.climberMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000); 

        this.climberMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000); 
        this.climberMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000); 

        this.climberMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000); 
        this.climberMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000); 
        
    }
    

    public void setDesiredSpeed1() {
       desiredSpeed1 = .8f;
    }
    public void setDesiredSpeed2() {
       desiredSpeed2 = -.8f;
    }


    public void setDesiredSpeedRev1() {
       desiredSpeed1 = -.3f;
    }
    public void setDesiredSpeedRev2() {
       desiredSpeed2 = .3f;
    }
    public void setDesiredSpeedZero1() {
       desiredSpeed1 = 0;
    }
    public void setDesiredSpeedZero2() {
       desiredSpeed2 = 0;
    }

    public void setDesiredSpeeds(final float f1, final float f2) {
        desiredSpeed1 = f1;
        desiredSpeed2 = f2;
    }

    @Override
    public void periodic() {

        
        
        climberMotor1.set(desiredSpeed1 * maxSpeed);

        climberMotor2.set(desiredSpeed2 * maxSpeed);
    }

}