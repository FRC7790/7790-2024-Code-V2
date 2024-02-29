 package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase
{
    private CANSparkMax pivotMotor1;
    private CANSparkMax pivotMotor2;
    private CANcoder pivotEncoder;

    private boolean isInitialized;

    private float angleOffset = 180;
    private float desiredAngle;
    private float angleMax;
    private float angleMin;
    private float speakerScoreAngle;
    private float ampScoreAngle;
    private float humanPickupAngle;
    private float groundPickupAngle;
    private float trapScoreAngle;
    private float homeStateAngle;
    private float shootAngle;
    private float movementAngle;
    //This should be used while arm is extending/retracting to clear frame.

    public float distanceValue;

    PIDController pid;

    public float NormalizeAngle(float angle)
    {
        float newAngle = angle - angleOffset;
         while(newAngle > 180)
         {
            newAngle -= 360;
         }

         while(newAngle < -180)
         {
            newAngle += 360;
         }
        return newAngle;
    }
    
    public Pivot() {
        
        //angle min -27

        this.isInitialized = false;
        this.angleMax = 61;
        this.angleMin = -44.85f;
        this.humanPickupAngle = 30;
        this.groundPickupAngle = -44.85f;
        //this.movementAngle = 0.0f;
        this.trapScoreAngle = 56;
        //this.speakerScoreAngle = 0.0f;
        this.homeStateAngle = -27;
        this.ampScoreAngle = 61;
        this.shootAngle = -31;
        
        this.pid = new PIDController(0.06, 0.0, 0.0);

        this.pivotEncoder = new CANcoder(15);

    
        this.pivotMotor1 = new CANSparkMax(20, CANSparkLowLevel.MotorType.kBrushless);
        this.pivotMotor2 = new CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushless);

        this.pivotMotor1.restoreFactoryDefaults();
        this.pivotMotor1.setIdleMode(IdleMode.kBrake);

        this.pivotMotor2.restoreFactoryDefaults();
        this.pivotMotor2.setIdleMode(IdleMode.kBrake);
        
        this.pivotMotor2.follow(this.pivotMotor1, true);

    }
    
    public void setDesiredAngle(final float desiredAngle) {
        this.desiredAngle = desiredAngle;
    }
    
    
     public void moveAmount(final float amount) {

        if (Math.abs(amount)<0.1){
            return;
        }

        float scale = 0.3f;

        float f = (float)MathUtil.clamp(this.desiredAngle + amount * scale, angleMin,angleMax);
        this.desiredAngle = f;
     }

    //manual move? ^
    
    public void setSpeakerScore() {
        this.setDesiredAngle(this.speakerScoreAngle);
    }

    public void setHumanPickup() {
        this.setDesiredAngle(this.humanPickupAngle);
    }
    
    public void setGroundPickup() {
        this.setDesiredAngle(this.groundPickupAngle);
    }
    
    public void setMovementAngle() {
        this.setDesiredAngle(this.movementAngle);
    }
    
    public void setHomeState() {
        this.setDesiredAngle(this.homeStateAngle);
    }

    public void setAmpScore() {
        this.setDesiredAngle(this.ampScoreAngle);
    }
    public void setShootAngle() {
        this.setDesiredAngle(this.shootAngle);
    }

    public Command setHomeCommand()
    {
        Command command = new InstantCommand(()-> this.setHomeState(), this);
        return command;
    }

    public Command setGroundCommand()
    {
        Command command = new InstantCommand(()-> this.setGroundPickup(), this);
        return command;
    }

    public Command setAmpScoreCommand()
    {
        Command command = new InstantCommand(()-> this.setAmpScore(), this);
        return command;
    }
    public Command setShootAngleCommand()
    {
        Command command = new InstantCommand(()-> this.setShootAngle(), this);
        return command;
    }

    public Command setHumanPickupCommand()
    {
        Command command = new InstantCommand(()-> this.setHumanPickup(), this);
        return command;
    }


     @Override
    public void periodic() {
        if (!this.isInitialized) {
            this.desiredAngle = NormalizeAngle((float)(this.pivotEncoder.getAbsolutePosition().getValue()*360));
            this.isInitialized = true;
        }
        final float maxoutput = 0.7f;
        final double output = MathUtil.clamp(this.pid.calculate(NormalizeAngle((float)(this.pivotEncoder.getAbsolutePosition().getValue()*360)), this.desiredAngle), -maxoutput, maxoutput);
        this.pivotMotor1.set(-output);
        SmartDashboard.putNumber("Arm Angle", NormalizeAngle((float)(this.pivotEncoder.getAbsolutePosition().getValue()*360))); 
        SmartDashboard.putNumber("Desired Angle", this.desiredAngle);
        SmartDashboard.putNumber("output", output);
    }
} 