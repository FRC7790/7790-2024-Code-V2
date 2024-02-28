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

    public static Command ampScoreRetractCommand(Extender extender, Pivot pivot)
    {
        Command command = pivot.setHomeCommand()
        .andThen(new WaitCommand(.8))
        .andThen(extender.homeStateCommand());

        command.addRequirements(pivot,extender);

        return command;
    }

public static Command harvestCommand(Extender extender, Shooter shooter, Pivot pivot)
    {
        Command command = extender.groundScoreCommand()
        .andThen(new WaitCommand(.8))
        .andThen(pivot.setGroundCommand())
        .andThen(shooter.startHarvestCommand());

        command.addRequirements(shooter,pivot,extender);

        return command;
    }

    public static Command retractHarvestCommand(Extender extender, Shooter shooter, Pivot pivot)
    {
        Command command = pivot.setHomeCommand()
        .andThen(shooter.stopHarvestCommand())
        .andThen(new WaitCommand(.8))
        .andThen(extender.homeStateCommand());

        command.addRequirements(shooter,pivot,extender);

        return command;
    }

    public static Command shootCommand(Extender extender, Shooter shooter, Pivot pivot)
    {
        Command command = pivot.setShootAngleCommand()
        .alongWith(shooter.startShooterCommand())
        .andThen(new WaitCommand(2))
        .andThen(shooter.shootCommand())
        .andThen(new WaitCommand(.8))
        .andThen(shooter.stopShooterCommand())
        .alongWith(shooter.indexStopCommand())
        .andThen(extender.homeStateCommand())
        .alongWith(pivot.setHomeCommand());

        command.addRequirements(shooter,pivot,extender);

        return command;
    }
}
