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
        .andThen(new WaitCommand(2))
        .andThen(shooter.startAmpShooterCommand())
        .andThen(new WaitCommand(1))
        .andThen(shooter.shootCommand())
        .andThen(new WaitCommand(1))
        .andThen(shooter.stopShooterCommand())
        .andThen(shooter.indexStopCommand());

        command.addRequirements(shooter,pivot,extender);

        return command;
    }

    public static Command ampScoreRetractCommand(Extender extender, Shooter shooter, Pivot pivot)
    {
        Command command = pivot.setHomeCommand()
        .andThen(shooter.stopShooterCommand())
        .andThen(shooter.stopHarvestCommand())
        .andThen(new WaitCommand(.6))
        .andThen(extender.homeStateCommand());

        command.addRequirements(pivot,extender);

        return command;
    }

public static Command harvestCommand(Extender extender, Shooter shooter, Pivot pivot)
    {
        Command command = pivot.setHomeCommand()
        .andThen(extender.groundScoreCommand())
        .andThen(new WaitCommand(.5))
        .andThen(pivot.setGroundCommand())
        .andThen(shooter.startHarvestCommand());
        
        //same logic as human pickup where leds are triggered here. Auto shutoff as well

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
        .andThen(new WaitCommand(.5))
        .andThen(shooter.startShooterCommand())
        .andThen(new WaitCommand(.75))
        .andThen(shooter.shootCommand())
        .andThen(new WaitCommand(.4))
        .andThen(shooter.stopShooterCommand())
        .andThen(shooter.indexStopCommand())
        .andThen(extender.homeStateCommand())
        .andThen(pivot.setHomeCommand());

        command.addRequirements(shooter,pivot,extender);

        return command;
    }

    public static Command humanPickupCommand(Extender extender, Shooter shooter, Pivot pivot)
    {
        Command command = pivot.setHumanPickupCommand()
        .andThen(new WaitCommand(.5))
        .andThen(extender.setHumanPickupCommand())
        .andThen(shooter.startHarvestCommand())

        //Add logic to start harvest to auto stop on after sensor triggered 
        //that would move it onto the next command

        .andThen(new WaitCommand(4))
        //this ^ goes away after auto note detection

        //require led to change here
        .andThen(shooter.stopHarvestCommand());

        command.addRequirements(shooter,pivot,extender);

        return command;
    }

    //Call Retract Amp to cancel Human Pickup
}
