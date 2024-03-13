    package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private int noteLoaded;
    private int navigateToSpeaker;
    private int endGameCelebration;
    private int shoot;
    private int standard;
    
    SerialPort ledInfo = new SerialPort(115200, SerialPort.Port.kOnboard);

    double lastRunTime = Timer.getMatchTime();
    
    private int  currentPattern;
    
    public LED() {
        //3,4,7 are coolest
        
        standard = 1;
        noteLoaded = 2;
        navigateToSpeaker = 0;
        endGameCelebration = 7;
        shoot = 0;
        currentPattern = standard;
    }
    public void setDesiredLight(int desiredLight) {
        this.currentPattern = (desiredLight);

       
    }

    public void noteLoaded() {
        this.setDesiredLight(this.noteLoaded);
    }


    public void setstandard() {
        this.setDesiredLight(this.standard);
    }

    public void setShoot() {
       // this.setDesiredLight(this.shoot);
    }

    @Override
    public void periodic() {

        boolean runTheTask;

        double curTime = Timer.getFPGATimestamp();
        double timeSinceLastRun = curTime - lastRunTime;

        if(timeSinceLastRun > .1f)
        {
            runTheTask = true;
        }
        else
        {
            runTheTask = false;
        }

        if(runTheTask)
        {
            

            String numString = String.valueOf(currentPattern);

            // System.out.println("l " + numString);

            ledInfo.writeString(numString);

            lastRunTime = curTime;
        }

    }

}    