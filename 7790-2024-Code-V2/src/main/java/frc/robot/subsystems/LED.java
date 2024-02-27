  package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private char desiredLight;
    private char noteLoaded;
    private char navigateToSpeaker;
    private char endGameCelebration;
    
    SerialPort ledInfo = new SerialPort(9600, SerialPort.Port.kOnboard);

    double lastRunTime = Timer.getFPGATimestamp();
    int currentPattern;
    public LED() {
        desiredLight = 'a';
        noteLoaded = 'b';
        navigateToSpeaker = 'c';
        endGameCelebration = 'z';
    }
    public void setDesiredLight(final char desiredLight) {
        this.desiredLight = (desiredLight);
    }

    public void noteLoaded() {
        this.setDesiredLight(this.noteLoaded);
    }

    public void setGroundPose() {
        this.setDesiredLight(this.navigateToSpeaker);
    }

    public void setHomeState() {
        this.setDesiredLight(this.endGameCelebration);
    }

    @Override
    public void periodic() {

        boolean runTheTask;

        double curTime = Timer.getFPGATimestamp();
        double timeSinceLastRun = curTime - lastRunTime;

        if(timeSinceLastRun > 3f)
        {
            runTheTask = true;
        }
        else
        {
            runTheTask = false;
        }

        if(runTheTask)
        {
            //do work
            currentPattern++;
            if(currentPattern > 4)
            {
                currentPattern = 0;
            }

            String numString = String.valueOf(currentPattern);

            ledInfo.writeString(numString);

            lastRunTime = curTime;
        }

    }

}  