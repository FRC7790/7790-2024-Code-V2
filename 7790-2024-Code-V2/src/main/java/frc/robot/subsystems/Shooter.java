package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.pathfinding.Aiming;

public class Shooter extends SubsystemBase
{
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    private CANSparkMax shooterMotor3;
    private CANSparkMax shooterMotor4;
    private CANSparkMax intakeMotor;
    private CANSparkMax indexMotor;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    private SparkPIDController shooterMotor1PID;
    private SparkPIDController shooterMotor2PID;
    private SparkPIDController shooterMotor3PID;
    private SparkPIDController shooterMotor4PID;

    private RelativeEncoder shooter1Encoder;
    private RelativeEncoder shooter2Encoder;
    private RelativeEncoder shooter3Encoder;
    private RelativeEncoder shooter4Encoder;

    public boolean isTriggered = false;

    
    private double setpoint1 = 0;
    private double setpoint2 = 0;
    private double setpoint3 = 0;
    private double setpoint4 = 0;
    
    private double dashboardMultiplier;
    DigitalInput noteSensor = new DigitalInput(9);

    LED led;

    Boolean isShooting = false;
     Boolean isReverse = false;
     Boolean isHarvesting = false;
    public Shooter(LED led) {

        this.led = led;
        this.shooterMotor1 = new CANSparkMax(30, CANSparkLowLevel.MotorType.kBrushless);
        this.shooterMotor2 = new CANSparkMax(31, CANSparkLowLevel.MotorType.kBrushless);
        this.shooterMotor3 = new CANSparkMax(32, CANSparkLowLevel.MotorType.kBrushless);
        this.shooterMotor4 = new CANSparkMax(33, CANSparkLowLevel.MotorType.kBrushless);
        this.indexMotor = new CANSparkMax(35, CANSparkLowLevel.MotorType.kBrushless);
        this.intakeMotor = new CANSparkMax(34, CANSparkLowLevel.MotorType.kBrushless);
        
        this.shooterMotor1.restoreFactoryDefaults();
        this.shooterMotor2.restoreFactoryDefaults();
        this.shooterMotor3.restoreFactoryDefaults();
        this.shooterMotor4.restoreFactoryDefaults();
        this.indexMotor.restoreFactoryDefaults();
        this.intakeMotor.restoreFactoryDefaults();

        this.shooterMotor1.setIdleMode(IdleMode.kCoast);
        this.shooterMotor2.setIdleMode(IdleMode.kCoast);
        this.shooterMotor3.setIdleMode(IdleMode.kCoast);
        this.shooterMotor4.setIdleMode(IdleMode.kCoast);
        this.indexMotor.setIdleMode(IdleMode.kCoast);
        this.intakeMotor.setIdleMode(IdleMode.kCoast);
        
        this.dashboardMultiplier = 1.00;



        this.indexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10000); 
        this.intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10000); 

        this.indexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000); 
        this.intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000); 

        this.indexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000); 
        this.intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000); 

        this.indexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000); 
        this.intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000); 
        

        this.shooterMotor1.setInverted(true);
        this.shooterMotor4.setInverted(true);



        shooterMotor1PID = shooterMotor1.getPIDController();

        shooterMotor2PID = shooterMotor2.getPIDController();

        shooterMotor3PID = shooterMotor3.getPIDController();

        shooterMotor4PID = shooterMotor4.getPIDController();


        shooter1Encoder = shooterMotor1.getEncoder();

        shooter2Encoder = shooterMotor2.getEncoder();

        shooter3Encoder = shooterMotor3.getEncoder();

        shooter4Encoder = shooterMotor4.getEncoder();

    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 11000;
        
    shooterMotor1PID.setP(kP);
    shooterMotor1PID.setI(kI);
    shooterMotor1PID.setD(kD);
    shooterMotor1PID.setIZone(kIz);
    shooterMotor1PID.setFF(kFF);
    shooterMotor1PID.setOutputRange(kMinOutput, kMaxOutput);

    shooterMotor2PID.setP(kP);
    shooterMotor2PID.setI(kI);
    shooterMotor2PID.setD(kD);
    shooterMotor2PID.setIZone(kIz);
    shooterMotor2PID.setFF(kFF);
    shooterMotor2PID.setOutputRange(kMinOutput, kMaxOutput);

    shooterMotor3PID.setP(kP);
    shooterMotor3PID.setI(kI);
    shooterMotor3PID.setD(kD);
    shooterMotor3PID.setIZone(kIz);
    shooterMotor3PID.setFF(kFF);
    shooterMotor3PID.setOutputRange(kMinOutput, kMaxOutput);

    shooterMotor4PID.setP(kP);
    shooterMotor4PID.setI(kI);
    shooterMotor4PID.setD(kD);
    shooterMotor4PID.setIZone(kIz);
    shooterMotor4PID.setFF(kFF);
    shooterMotor4PID.setOutputRange(kMinOutput, kMaxOutput);


    SmartDashboard.putNumber("Shooter Multiplier", dashboardMultiplier);
    
    }
    public void startShooter() {
        //shooterMotor1.set(-.45);
        //shooterMotor3.set(.4);
        
        double speed = 7000;

        dashboardMultiplier = SmartDashboard.getNumber("Shooter Multiplier", 1);

        speed = speed * dashboardMultiplier;

        if (Aiming.longShot) {
            speed = 8200 * dashboardMultiplier;
        }

        if (Aiming.shortShot) {
            //speed = 6000 * dashboardMultiplier;
            speed = 7000 * dashboardMultiplier;
        }

        setpoint1 = speed;
        setpoint2 = speed;
        setpoint3 = speed;
        setpoint4 = speed;


    }

    public void startPass() {
        setpoint1 = 5000;
        setpoint2 = 5000;
        setpoint3 = 5000;
        setpoint4 = 5000;
    }
        public void startAmpShooter() {
        setpoint1 = 2000;
        setpoint2 = 2000;
        setpoint3 = 2000;
        setpoint4 = 2000;
    }
    public void stopShooter() {
        setpoint1 = 0;
        setpoint2 = 0;
        setpoint3 = 0;
        setpoint4 = 0;
    }


    public void harvest() {

        isShooting = false;
        isReverse = false;
        isHarvesting = true;
        intakeMotor.set(.5);
        indexMotor.set(.5);
        led.setstandard();
    }

    public void forward() {

        isShooting = false;
        isReverse = true;
        isHarvesting = false;
        intakeMotor.set(.35);
        indexMotor.set(.35);
    }

    public void harvestStop() {
        isShooting = false;
        isReverse = false;
        isHarvesting = false;
        intakeMotor.set(0);
        indexMotor.set(0);
    }
    public void harvestReverse() {
         isShooting = false;
         isReverse = true;
         isHarvesting = false;
        intakeMotor.set(-.4);
        indexMotor.set(-.4);
    }
    public void shoot() {
        isShooting = true;
        isReverse = false;
        isHarvesting = false;
        intakeMotor.set(.35);
        indexMotor.set(.5);

         led.setShoot();
    }
    
      public void indexStop() {
         isShooting = false;
         isReverse = false;
         isHarvesting = false;
        indexMotor.set(0);
        intakeMotor.set(0);
      }
    public void setisAutonHarvest(){
        isHarvesting = true;
    }
public void setisAutonHarvestStop(){
        isHarvesting = false;
    }
      public Command startShooterCommand()
{
    Command command = new InstantCommand(()->startShooter(), this);
    return command;
}

public Command shootCommand()
{
    Command command = new InstantCommand(()->shoot(), this);
    return command;
}

public Command indexStopCommand()
{
    Command command = new InstantCommand(()->indexStop(), this);
    return command;
}

public Command startAmpShooterCommand()
{
    Command command = new InstantCommand(()->startAmpShooter(), this);
    return command;
}

public Command stopShooterCommand()
{
    Command command = new InstantCommand(()->stopShooter(), this);
    return command;
}

public Command startHarvestCommand()
{
    Command command = new InstantCommand(()->harvest(), this);
    
    return command;
}

public Command stopHarvestCommand()
{
    Command command = new InstantCommand(()->harvestStop() , this);
    return command;
}
public Command forwardCommand()
{
    Command command = new InstantCommand(()->forward() , this);
    return command;
}
public Command startPassCommand()
{
    Command command = new InstantCommand(()->startPass() , this);
    return command;
}



    @Override
    public void periodic() {

        isTriggered = !noteSensor.get();

        SmartDashboard.putBoolean("noteSensor", isTriggered);
        if(isTriggered)
        {
           
            led.noteLoaded();
            if(isHarvesting){
                harvestStop();               
            }
        }
        else
        {
            led.setstandard();
        }
        
        
        shooterMotor1PID.setReference(setpoint1, CANSparkMax.ControlType.kVelocity);

        shooterMotor2PID.setReference(setpoint2, CANSparkMax.ControlType.kVelocity);

        shooterMotor3PID.setReference(setpoint3, CANSparkMax.ControlType.kVelocity);

        shooterMotor4PID.setReference(setpoint4, CANSparkMax.ControlType.kVelocity);

        }
    }