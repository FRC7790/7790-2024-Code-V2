package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    private CANSparkMax shooterMotor3;
    private CANSparkMax shooterMotor4;
    private CANSparkMax intakeMotor;
    private CANSparkMax indexMotor;

    private RelativeEncoder shooter1Encoder;
    private RelativeEncoder shooter2Encoder;
    private RelativeEncoder shooter3Encoder;
    private RelativeEncoder shooter4Encoder;

    public boolean isTriggered = false;

    private boolean isShooting = false;
    
    private double setpoint1 = 0;
    private double setpoint2 = 0;
    private double setpoint3 = 0;
    private double setpoint4 = 0;
    
    DigitalInput noteSensor = new DigitalInput(9);


    BangBangController controller1 = new BangBangController();
    BangBangController controller2 = new BangBangController();
    BangBangController controller3 = new BangBangController();
    BangBangController controller4 = new BangBangController();

    LED led;
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
        
        this.shooterMotor1.setInverted(true);
        this.shooterMotor4.setInverted(true);

        shooter1Encoder = shooterMotor1.getEncoder();

        shooter2Encoder = shooterMotor2.getEncoder();

        shooter3Encoder = shooterMotor3.getEncoder();

        shooter4Encoder = shooterMotor4.getEncoder();


    }
    public void startShooter() {
        //shooterMotor1.set(-.45);
        //shooterMotor3.set(.4);
        this.shooterMotor1.setIdleMode(IdleMode.kCoast);
        this.shooterMotor2.setIdleMode(IdleMode.kCoast);
        this.shooterMotor3.setIdleMode(IdleMode.kCoast);
        this.shooterMotor4.setIdleMode(IdleMode.kCoast);
        setpoint1 = 2000;
        setpoint2 = 2000;
        setpoint3 = 2000;
        setpoint4 = 2000;
        isShooting = true;
    }
        public void startAmpShooter() {
        this.shooterMotor1.setIdleMode(IdleMode.kCoast);
        this.shooterMotor2.setIdleMode(IdleMode.kCoast);
        this.shooterMotor3.setIdleMode(IdleMode.kCoast);
        this.shooterMotor4.setIdleMode(IdleMode.kCoast);
        setpoint1 = 200;
        setpoint2 = 200;
        setpoint3 = 200;
        setpoint4 = 200;
        isShooting = true;
    }
    public void stopShooter() {
        //shooterMotor1.set(0);
        //shooterMotor3.set(0);
        this.shooterMotor1.setIdleMode(IdleMode.kBrake);
        this.shooterMotor2.setIdleMode(IdleMode.kBrake);
        this.shooterMotor3.setIdleMode(IdleMode.kBrake);
        this.shooterMotor4.setIdleMode(IdleMode.kBrake);
        setpoint1 = 0;
        setpoint2 = 0;
        setpoint3 = 0;
        setpoint4 = 0;
        isShooting = false;
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

    public void harvest() {

        intakeMotor.set(.3);
        indexMotor.set(.4);
        led.setstandard();
     
    }

    public void harvestStop() {
        intakeMotor.set(0);
        indexMotor.set(0);
    }
    public void harvestReverse() {
        intakeMotor.set(-.2);
        indexMotor.set(-.2);
    }
    public void shoot() {
        intakeMotor.set(.3);
        indexMotor.set(.4);

         led.setShoot();
    }
    
      public void indexStop() {
        indexMotor.set(0);
        intakeMotor.set(0);
      }
    
    @Override
    public void periodic() {

        isTriggered = !noteSensor.get();

        if(isTriggered)
        {
            led.noteLoaded();
        }
        else
        {
            led.setstandard();
        }
        
        float bangBangMultiplier = 1f;

        double speed1 = shooter1Encoder.getVelocity();
        double speed2 = shooter2Encoder.getVelocity();
        double speed3 = shooter3Encoder.getVelocity();
        double speed4 = shooter4Encoder.getVelocity();






        double motor1Speed = controller1.calculate(speed1, setpoint1);
        double motor2Speed = controller2.calculate(speed2, setpoint2);
        double motor3Speed = controller3.calculate(speed3, setpoint3);
        double motor4Speed = controller4.calculate(speed4, setpoint4);

        motor1Speed = motor1Speed * bangBangMultiplier;
        motor2Speed = motor2Speed * bangBangMultiplier;
        motor3Speed = motor3Speed * bangBangMultiplier;
        motor4Speed = motor4Speed * bangBangMultiplier;
    
        if (!isShooting) {
            motor1Speed = 0;
            motor2Speed = 0;
            motor3Speed = 0;
            motor4Speed = 0;
        }

        shooterMotor1.set(motor1Speed);
        
        shooterMotor2.set(motor2Speed);

        shooterMotor3.set(motor3Speed);
        
        shooterMotor4.set(motor4Speed);

        }
    }