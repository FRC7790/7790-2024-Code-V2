package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

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
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    private RelativeEncoder extender1encoder;
    private SparkPIDController m_pidController;

    
    public Extender() {
        this.desiredPose = 0.0f;
        this.poseMax = 35.0f;
        this.poseMin = 0.0f;
        this.humanPickupPose = 20.0f;
        this.groundPickupPose = -24.5f;
        this.movementPose = 0.0f;
        this.trapScorePose = 0.0f;
        this.speakerScorePose = 0.0f;
        this.homeStatePose = 0.0f;

        this.extenderMotor1 = new CANSparkMax(00, CANSparkLowLevel.MotorType.kBrushless);
        this.extenderMotor2 = new CANSparkMax(00, CANSparkLowLevel.MotorType.kBrushless);
        
        this.extenderMotor2.follow(this.extenderMotor1, true);

        extender1encoder = extenderMotor1.getEncoder();

        //Need to add logic to keep motors in sync with each other!!!
        m_pidController = extenderMotor1.getPIDController();
        // PID coefficients
        kP = 0.1;
        kI = 1e-4;
        kD = 1;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;
    
        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    
    }
    
    public void setDesiredPose(final float desiredPose) {
        this.desiredPose = (desiredPose);
    }

    public void setHumanPickup() {
        this.setDesiredPose(this.humanPickupPose);
        m_pidController.setReference(desiredPose, CANSparkMax.ControlType.kPosition);
    }
    
    public void setGroundPose() {
        this.setDesiredPose(this.groundPickupPose);
        m_pidController.setReference(desiredPose, CANSparkMax.ControlType.kPosition);
    }
    
    public void setMovementPose() {
        this.setDesiredPose(this.movementPose);
        m_pidController.setReference(desiredPose, CANSparkMax.ControlType.kPosition);
    }
    
    public void setHomeState() {
        this.setDesiredPose(this.homeStatePose);
        m_pidController.setReference(desiredPose, CANSparkMax.ControlType.kPosition);
    }
}