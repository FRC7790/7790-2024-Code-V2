package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase
{
    private CANSparkMax extenderMotor1;
    private CANSparkMax extenderMotor2;
    private float desiredPosition;
    private float extenderMax;
    private float extenderMin;
    private float speakerScorePose;
    private float ampScorePose;
    private float climbPose;
    private float groundPickupPose;
    private float trapScorePose;
    private float homeStatePose;

    PIDController pid;
    PIDController pid2;

    private RelativeEncoder extender1Encoder;
    private RelativeEncoder extender2Encoder;

    private LED led;


    public Extender(LED led) {
        this.led = led;
        this.desiredPosition = 0.0f;

        //max 44.5f

        this.extenderMax = 45f;
        this.extenderMin = 0.5f;
        this.climbPose = 44.5f;
        this.groundPickupPose = 44.5f;
        this.trapScorePose = 44.5f;
        this.speakerScorePose = 0.5f;
        this.homeStatePose = 0.5f;
        this.ampScorePose = 44.5f;

        this.extenderMotor1 = new CANSparkMax(22, CANSparkLowLevel.MotorType.kBrushless);
        this.extenderMotor2 = new CANSparkMax(23, CANSparkLowLevel.MotorType.kBrushless);

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

        this.pid = new PIDController(0.2, 0.0, 0.0);
    
    }
    
    public void setDesiredPosition(final float desiredPose) {

        float newDesiredPose = (float)MathUtil.clamp(desiredPose, extenderMin, extenderMax);



        this.desiredPosition = (newDesiredPose);
    }

    public void setFullRetract(){

        this.desiredPosition = 0;
    }

    public void extendAmount(final float amount) {

        
        if (Math.abs(amount)<0.2){
            return;
        }

        float scale = 0.2f;
        this.setDesiredPosition(this.desiredPosition + amount * scale);
     }

    public void setClimb() {
        this.setDesiredPosition(this.climbPose);     
        
    }
    public void setGroundPose() {
        this.setDesiredPosition(this.groundPickupPose);       
    }
    public void setHomeState() {
        this.setDesiredPosition(this.homeStatePose);
    }

    public void setAmpPose() {
        this.setDesiredPosition(this.ampScorePose);       
    }

    //Commands
    public Command homeStateCommand()
    {
        Command command = new InstantCommand(()-> this.setHomeState(), this);
        return command;
    }
    public Command fullRetractCommand()
    {
        Command command = new InstantCommand(()-> this.setFullRetract(), this);
        return command;
    }

    public Command ampScoreCommand()
    {
        Command command = new InstantCommand(()-> this.setAmpPose(), this);
        return command;
    }

    public Command groundScoreCommand()
    {
        Command command = new InstantCommand(()-> this.setGroundPose(), this);
        return command;
    }

    public Command setClimbCommand()
    {
        Command command = new InstantCommand(()-> this.setClimb(), this);
        return command;
    }
    
    @Override
    public void periodic() {

        final float maxoutput = 0.7f;

        //final float maxoutput = 0.1f;

        double pos1 = extender1Encoder.getPosition();
        double pos2 = extender2Encoder.getPosition();

        //System.out.println(pos1);
        //System.out.println(pos2);

        final double output = MathUtil.clamp(this.pid.calculate(pos1, this.desiredPosition), -maxoutput, maxoutput);
        extenderMotor1.set(output);

      //  System.out.println(output);

        final double output2 = MathUtil.clamp(this.pid.calculate(pos2, this.desiredPosition), -maxoutput, maxoutput);
        extenderMotor2.set(output2);

      //  System.out.println(output2);

    }

}