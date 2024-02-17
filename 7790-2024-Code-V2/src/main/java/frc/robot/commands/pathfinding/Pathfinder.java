package frc.robot.commands.pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class Pathfinder {

    public static void driveToPose(int buttonNum) {
        // Since we are using a holonomic drivetrain, the rotation component of this
        // pose
        // represents the goal holonomic rotation
        float x = 0;
        float y = 0;

        switch (buttonNum) {
            case 1:
                x = 0;
                y = 2;
                break;
            case 2:
                x = 1;
                y = 2;
                break;
            case 3:
                x = 2;
                y = 2;
                break;
            case 4:
                x = 0;
                y = 1;
                break;
            case 5:
                x = 1;
                y = 1;
                break;
            case 6:
                x = 2;
                y = 1;
                break;
            case 7:
                x = 0;
                y = 0;
                break;
            case 8:
                x = 1;
                y = 0;
                break;
            case 9:
                x = 2;
                y = 0;
                break;
        
            default:
                break;
        }
        Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(180));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        );

    }

}
