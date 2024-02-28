package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class CommandFactory {
    public static Command ampScoreCommand(Extender extender, Shooter shooter, Pivot pivot)
    {
        Command command = extender.ampScoreCommand()
        .andThen(pivot.setAmpScoreCommand())
        .andThen(new WaitCommand(3))
        .andThen(shooter.startAmpShooterCommand())
        .andThen(new WaitCommand(1))
        .andThen(shooter.shootCommand())
        .andThen(new WaitCommand(1))
        .andThen(shooter.stopShooterCommand())
        .andThen(shooter.indexStopCommand());

        command.addRequirements(shooter,pivot,extender);

        return command;
    }
}
