package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

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

    PIDController pid;
    PIDController pid2;

    private RelativeEncoder extender1Encoder;
    private RelativeEncoder extender2Encoder;

    
    public Extender() {
        this.desiredPosition = 0.0f;
        this.poseMax = 10.0f;
        this.poseMin = 0.5f;
        this.humanPickupPose = 10.0f;
        this.groundPickupPose = 2.0f;
        this.trapScorePose = 0.0f;
        this.speakerScorePose = 0.0f;
        this.homeStatePose = 0.0f;

        this.extenderMotor1 = new CANSparkMax(60, CANSparkLowLevel.MotorType.kBrushless);
        this.extenderMotor2 = new CANSparkMax(61, CANSparkLowLevel.MotorType.kBrushless);

        this.extenderMotor1.restoreFactoryDefaults();
        this.extenderMotor1.setIdleMode(IdleMode.kBrake);

        this.extenderMotor2.restoreFactoryDefaults();
        this.extenderMotor2.setIdleMode(IdleMode.kBrake);
        
        this.extenderMotor2.setInverted(true);
    
        extender1Encoder = extenderMotor1.getEncoder();
        extender1Encoder.setPosition(0);
        extender1Encoder.setPositionConversionFactor(1);

        extender2Encoder = extenderMotor2.getEncoder();
        extender2Encoder.setPosition(0);
        extender2Encoder.setPositionConversionFactor(1);

        this.pid = new PIDController(0.07, 0.0, 0.0);
    
    }
    
    public void setDesiredPosition(final float desiredPose) {

        float newDesiredPose = (float)MathUtil.clamp(desiredPose, poseMin, poseMax);



        this.desiredPosition = (newDesiredPose);
    }

    public void extendAmount(final float amount) {

        
        if (Math.abs(amount)<0.1){
            return;
        }

        float scale = 0.1f;
        this.setDesiredPosition(this.desiredPosition + amount * scale);
     }

    public void setHumanPickup() {
        this.setDesiredPosition(this.humanPickupPose);       
    }
    public void setGroundPose() {
        this.setDesiredPosition(this.groundPickupPose);       
    }
    public void setHomeState() {
        this.setDesiredPosition(this.homeStatePose);
    }
    @Override
    public void periodic() {

        final float maxoutput = 0.1f;

        double pos1 = extender1Encoder.getPosition();
        double pos2 = extender2Encoder.getPosition();

        final double output = MathUtil.clamp(this.pid.calculate(pos1, this.desiredPosition), -maxoutput, maxoutput);
        extenderMotor1.set(output);

        final double output2 = MathUtil.clamp(this.pid.calculate(pos2, this.desiredPosition), -maxoutput, maxoutput);
        extenderMotor2.set(output2);
    }

}