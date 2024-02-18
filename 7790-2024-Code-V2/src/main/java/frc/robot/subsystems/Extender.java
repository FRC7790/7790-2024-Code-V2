package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase
{
    private CANSparkMax extenderMotor1;
    private CANSparkMax extenderMotor2;
    private float desiredPose;
    private float poseMax;
    private float poseMin;
    private float speakerScorePose;
    private float ampScorePose;
    private float humanPickupPose;
    private float groundPickupPose;
    private float trapScorePose;
    private float homeStatePose;

    private float movementPose;
    //This should be used while arm is extending/retracting to clear frame.

    public float distanceValue;

    PIDController pid;
    
    public Extender() {
        this.desiredPose = 0.0f;
        this.poseMax = 35.0f;
        this.poseMin = -52.0f;
        this.humanPickupPose = 20.0f;
        this.groundPickupPose = -24.5f;
        this.movementPose = 0.0f;
        this.trapScorePose = 0.0f;
        this.speakerScorePose = 0.0f;
        this.homeStatePose = 0.0f;
        
        this.pid = new PIDController(0.03, 0.0, 0.0);

        this.extenderMotor1 = new CANSparkMax(00, CANSparkLowLevel.MotorType.kBrushless);
        this.extenderMotor2 = new CANSparkMax(00, CANSparkLowLevel.MotorType.kBrushless);
        
        this.extenderMotor2.follow(this.extenderMotor1, true);

    }
    
    public void setDesiredPose(final float desiredPose) {
        this.desiredPose = (float)MathUtil.clamp(desiredPose, this.poseMin, this.poseMax);
    }
    
   /*  public void extendAmount(final float amountExtend) {
        this.setDesiredPose(this.desiredPose + amountExtend);
    } */

    public void setHumanPickup() {
        this.setDesiredPose(this.humanPickupPose);
    }
    
    public void setGroundPose() {
        this.setDesiredPose(this.groundPickupPose);
    }
    
    public void setMovementPose() {
        this.setDesiredPose(this.movementPose);
    }
    
    public void setHomeState() {
        this.setDesiredPose(this.homeStatePose);
    }
}