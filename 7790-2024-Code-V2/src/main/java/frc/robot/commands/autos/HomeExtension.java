package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class HomeExtension extends SequentialCommandGroup
{
    public HomeExtension(final Pivot s_Pivot, final Extender s_Extender, final Shooter s_Shooter) {
        super(new Command[0]);
        this.addCommands(new InstantCommand(() -> s_Shooter.harvestStop(), new Subsystem[0]), 

        new WaitCommand(0.1), 

        new InstantCommand(() -> s_Extender.setHomeState(), new Subsystem[0]),
        
        new InstantCommand(() -> s_Pivot.setMovementAngle(), new Subsystem[0]));
    }
}