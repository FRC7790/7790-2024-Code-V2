package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UnicornHorn extends SubsystemBase {

    Servo unicornServo;

    public UnicornHorn() {
        unicornServo = new Servo(9);
        unicornServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        unicornServo.setSpeed(-1);
    }

    public void unicornRetract() {
        unicornServo.setSpeed(-1);

    }

    public void unicornExtend() {
        unicornServo.setSpeed(1);
    }
}