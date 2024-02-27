 package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkLowLevel;
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
    private float desiredPosition;
    private float poseMax;
    private float poseMin;
    private float nonRetracted;
    private float retracted;


    PIDController pid;
    PIDController pid2;

    private RelativeEncoder climber1Encoder;
    private RelativeEncoder climber2Encoder;

    
    public Climber() {
        this.desiredPosition = 0.0f;
        this.poseMax = 10.0f;
        this.poseMin = 0.0f;
        this.nonRetracted = 10.0f;
        this.retracted = 2.0f;

        this.climberMotor1 = new CANSparkMax(22, CANSparkLowLevel.MotorType.kBrushless);
        this.climberMotor2 = new CANSparkMax(23, CANSparkLowLevel.MotorType.kBrushless);

        this.climberMotor1.restoreFactoryDefaults();
        this.climberMotor1.setIdleMode(IdleMode.kBrake);

        this.climberMotor2.restoreFactoryDefaults();
        this.climberMotor2.setIdleMode(IdleMode.kBrake);
        
    
        climber1Encoder = climberMotor1.getEncoder();
        climber1Encoder.setPosition(0);
        climber1Encoder.setPositionConversionFactor(1);

        climber2Encoder = climberMotor2.getEncoder();
        climber2Encoder.setPosition(0);
        climber2Encoder.setPositionConversionFactor(1);

        this.pid = new PIDController(0.07, 0.0, 0.0);
    }
    
    public void setDesiredClimb(final float desiredClimb) {
        float newDesiredClimb = (float)MathUtil.clamp(desiredClimb, poseMin, poseMax);



        this.desiredPosition = (newDesiredClimb);
    }

    public void climbAmount(final float amount) {

        
        if (Math.abs(amount)<0.1){
            return;
        }

        float scale = 0.1f;
        this.setDesiredClimb(this.desiredPosition + amount * scale);
     }

    public void setClimbToTrap() {
        this.setDesiredClimb(this.retracted);       
    }
    
    public void setHomeClimb() {
        this.setDesiredClimb(this.nonRetracted);
    }


    @Override
    public void periodic() {

        
        final float maxoutput = 0.03f;

        double pos1 = climber1Encoder.getPosition();
        double pos2 = climber2Encoder.getPosition();

        final double output = MathUtil.clamp(this.pid.calculate(pos1, this.desiredPosition), -maxoutput, maxoutput);
        climberMotor1.set(output);

        final double output2 = MathUtil.clamp(this.pid.calculate(pos2, this.desiredPosition), -maxoutput, maxoutput);
        climberMotor2.set(output2);
    }

}
