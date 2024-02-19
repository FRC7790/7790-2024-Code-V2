package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase
{
    private CANSparkMax extenderMotor1;
    private CANSparkMax extenderMotor2;
    private float desiredPosition;
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
    PIDController pid;
    public float distanceValue;

    private RelativeEncoder extender1encoder;

    
    public Extender() {
        this.desiredPosition = 0.0f;
        this.poseMax = 10.0f;
        this.poseMin = 0.0f;
        this.humanPickupPose = 10.0f;
        this.groundPickupPose = 5.0f;
        this.trapScorePose = 0.0f;
        this.speakerScorePose = 0.0f;
        this.homeStatePose = 0.0f;

        this.extenderMotor1 = new CANSparkMax(40, CANSparkLowLevel.MotorType.kBrushless);
        this.extenderMotor2 = new CANSparkMax(41, CANSparkLowLevel.MotorType.kBrushless);
        
        this.extenderMotor2.follow(this.extenderMotor1, true);

        extender1encoder = extenderMotor1.getEncoder();

        this.pid = new PIDController(0.03, 0.0, 0.0);
    }
    
    public void setDesiredPosition(final float desiredPose) {
        this.desiredPosition = (desiredPose);
    }

    public void setHumanPickup() {
        this.setDesiredPosition(this.humanPickupPose);       
    }

    //lavantense ovegente
    public void setGroundPose() {
        this.setDesiredPosition(this.groundPickupPose);       
    }
    public void setHomeState() {
        this.setDesiredPosition(this.homeStatePose);
    }
    @Override
    public void periodic() {

        final float maxoutput = 0.5f;
        final double output = MathUtil.clamp(this.pid.calculate(extender1encoder.getPosition(),this.desiredPosition), -maxoutput, maxoutput);
        extenderMotor1.set(output);

        System.out.println(output);
        System.out.println();
        System.out.println(desiredPosition);
        System.out.println();
        System.out.println(extender1encoder.getPosition());

    }

}