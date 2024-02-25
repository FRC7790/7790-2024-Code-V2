package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private char desiredLight;
    private char noteLoaded;
    private char navigateToSpeaker;
    private char endGameCelebration;
    
    SerialPort ledInfo = new SerialPort(50, null);

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

    }

}