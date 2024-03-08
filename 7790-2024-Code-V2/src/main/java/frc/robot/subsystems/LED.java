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
    
    SerialPort ledInfo = new SerialPort(9600, SerialPort.Port.kOnboard);

    double lastRunTime = Timer.getMatchTime();
    
    private int  currentPattern;
    
    public LED() {
        //3,4,7 are coolest
        
        standard = 7;
        noteLoaded = 4;
        navigateToSpeaker = 0;
        endGameCelebration = 1;
        shoot = 3;
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
        this.setDesiredLight(this.shoot);
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
            

            String numString = String.valueOf(currentPattern);

             System.out.println("l " + numString);

            ledInfo.writeString(numString);

            lastRunTime = curTime;
        }

    }

}    