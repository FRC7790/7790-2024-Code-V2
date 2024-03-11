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
import frc.robot.commands.pathfinding.Aiming;

public class Pivot extends SubsystemBase
{
    private CANSparkMax pivotMotor1;
    private CANSparkMax pivotMotor2;
    private CANcoder pivotEncoder;

    private boolean isInitialized;

    private float angleOffset =-2;
    private float desiredAngle;
    private float angleMax;
    private float angleMin;
    private float speakerScoreAngle;
    private float ampScoreAngle;
    private float climbAngle;
    private float groundPickupAngle;
    private float trapScoreAngle;
    private float homeStateAngle;
    private float shootAngle;
    private float movementAngle;

    private float distanceBasedShootAngle;
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

        // Shoot Medium 18.24

        //17.44 long

        this.isInitialized = false;
        this.angleMax = 68;
        this.angleMin = -46.1f;
        this.climbAngle = 27;
        this.groundPickupAngle = -46f;
        //this.movementAngle = 0.0f;
        this.trapScoreAngle = 56;
        //this.speakerScoreAngle = 0.0f;
        this.homeStateAngle = -27;
        this.ampScoreAngle = 48f;
        this.shootAngle = -27;
        
        this.distanceBasedShootAngle = -27;
        // default to home state
        
        this.pid = new PIDController(0.08, 0.0, 0.0);

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

        if (Math.abs(amount)<0.2){
            return;
        }

        float scale = 0.2f;

        float f = (float)MathUtil.clamp(this.desiredAngle + amount * scale, angleMin,angleMax);

       //disabled after first match jackson

       //float f = (float)this.desiredAngle + amount * scale;


        this.desiredAngle = f;
     }

    //manual move? ^
    
    public void setSpeakerScore() {
        this.setDesiredAngle(this.speakerScoreAngle);
    }

    public void setClimbAngle() {
        this.setDesiredAngle(this.climbAngle);
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

    public Command setClimbAngleCommand()
    {
        Command command = new InstantCommand(()-> this.setClimbAngle(), this);
        return command;
    }


     @Override
    public void periodic() {
        if (!this.isInitialized) {
            this.desiredAngle = NormalizeAngle((float)(this.pivotEncoder.getAbsolutePosition().getValue()*360));
            this.isInitialized = true;
        }

        if(Aiming.isAiming){
           this.desiredAngle = Aiming.angle;
        }
        // Max was 0.7f
        final float maxoutput = 0.7f;
        final double output = MathUtil.clamp(this.pid.calculate(NormalizeAngle((float)(this.pivotEncoder.getAbsolutePosition().getValue()*360)), this.desiredAngle), -maxoutput, maxoutput);
        this.pivotMotor1.set(-output);
        SmartDashboard.putNumber("Arm Angle", NormalizeAngle((float)(this.pivotEncoder.getAbsolutePosition().getValue()*360))); 
        SmartDashboard.putNumber("Desired Angle", this.desiredAngle);
        SmartDashboard.putNumber("output", output);
    }
} 