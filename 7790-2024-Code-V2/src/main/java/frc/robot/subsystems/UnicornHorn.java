package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UnicornHorn extends SubsystemBase
{

Servo unicornServo = new Servo(9);

    public void unicornRetract() {
        unicornServo.set(0);
    }
    public void unicornExtend() {
        unicornServo.set(1);
    }
}