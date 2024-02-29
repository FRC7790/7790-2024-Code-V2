    package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private String desiredLight;
    private String noteLoaded;
    private String navigateToSpeaker;
    private String endGameCelebration;
    private String shoot;
    
    SerialPort ledInfo = new SerialPort(9600, SerialPort.Port.kOnboard);

    double lastRunTime = Timer.getFPGATimestamp();
    
    public LED() {
        desiredLight = "1";
        noteLoaded = "2";
        navigateToSpeaker = "3";
        endGameCelebration = "4";
        shoot = "5";
    }
    public void setDesiredLight(String desiredLight) {
        this.desiredLight = (desiredLight);
    }

    public void noteLoaded() {
        this.setDesiredLight(this.noteLoaded);
    }

    public void setGroundPose() {
        this.setDesiredLight(this.navigateToSpeaker);
    }
     public void setShoot() {
        this.setDesiredLight(this.shoot);
    }

    public void setHomeState() {
        this.setDesiredLight(this.endGameCelebration);
    }

    @Override
    public void periodic() {

        boolean runTheTask;

        double curTime = Timer.getFPGATimestamp();
        double timeSinceLastRun = curTime - lastRunTime;

        if(timeSinceLastRun > 0.5f)
        {
            runTheTask = true;
        }
        else
        {
            runTheTask = false;
        }

        if(runTheTask)
        {
            
            ledInfo.writeString(desiredLight);

            lastRunTime = curTime;
        }

    }

}    