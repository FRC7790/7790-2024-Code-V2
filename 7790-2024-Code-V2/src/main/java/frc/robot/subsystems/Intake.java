// 
// Decompiled by Procyon v0.5.36
// 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
    private CANSparkMax intakeMotor1;
    private CANSparkMax intakeMotor2;
    
    public Intake() {
      //  this.intakeMotor1 = new CANSparkMax(20, CANSparkLowLevel.MotorType.kBrushless);
     //   this.intakeMotor2 = new CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushless);
    }
}
