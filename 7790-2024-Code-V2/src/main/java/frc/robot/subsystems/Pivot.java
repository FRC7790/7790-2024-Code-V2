package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase
{
    private CANSparkMax pivotMotor1;
    private CANSparkMax pivotMotor2;
    private CANcoder pivotEncoder;

    private boolean isInitialized;

    private float desiredAngle;
    private float angleOffset;
    private float angleMax;
    private float angleMin;
    private float speakerScoreAngle;
    private float ampScoreAngle;
    private float humanPickupAngle;
    private float groundPickupAngle;
    private float trapScoreAngle;
    private float homeStateAngle;

    private float movementAngle;
    //This should be used while arm is extending/retracting to clear frame.

    public float distanceValue;

    PIDController pid;
    
    public Pivot() {
        this.desiredAngle = 0.0f;
        this.isInitialized = false;
        this.angleOffset = 255.0f;
        this.angleMax = 35.0f;
        this.angleMin = -52.0f;
        this.humanPickupAngle = 20.0f;
        this.groundPickupAngle = -24.5f;
        this.movementAngle = 0.0f;
        this.trapScoreAngle = 0.0f;
        this.speakerScoreAngle = 0.0f;
        this.homeStateAngle = 0.0f;
        
        this.pid = new PIDController(0.03, 0.0, 0.0);

        this.pivotEncoder = new CANcoder(60);
        this.pivotMotor1 = new CANSparkMax(61, CANSparkLowLevel.MotorType.kBrushless);
        this.pivotMotor2 = new CANSparkMax(62, CANSparkLowLevel.MotorType.kBrushless);

        this.pivotMotor2.follow(this.pivotMotor1, true);

    }
    
    public void setDesiredAngle(final float desiredAngle) {
        this.desiredAngle = (float)MathUtil.clamp(desiredAngle, this.angleMin, this.angleMax);
    }
    
   /*  public void moveAmount(final float amount) {
        this.setDesiredAngle(this.desiredAngle + amount);
    } */

    //manual move? ^
    
    public void setSpeakerScore() {
        this.setDesiredAngle(this.speakerScoreAngle + distanceValue);
    }
    
    //Speaker Score needs to be adjusted using another value for distance.
    //The starting point should be its highest angle.

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
    


     @Override
    public void periodic() {
        if (!this.isInitialized) {
            this.desiredAngle = (float)(this.pivotEncoder.getAbsolutePosition().getValue() - this.angleOffset);
            this.isInitialized = true;
        }
        final float maxoutput = 0.2f;
        final double output = MathUtil.clamp(this.pid.calculate(this.pivotEncoder.getAbsolutePosition().getValue() - this.angleOffset, this.desiredAngle), -maxoutput, maxoutput);
        this.pivotMotor1.set(-output);
        SmartDashboard.putNumber("Arm Angle", this.pivotEncoder.getAbsolutePosition().getValue() - this.angleOffset);
        SmartDashboard.putNumber("Desired Angle", this.desiredAngle);
        SmartDashboard.putNumber("output", output);
    }
}