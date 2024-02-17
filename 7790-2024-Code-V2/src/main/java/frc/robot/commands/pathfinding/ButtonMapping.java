package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ButtonMapping {
    public static Pose2d buttonToPose(int buttonNum){
        float x = 0;
        float y = 0;

        switch (buttonNum) {
            case 1:
                x = 2;
                y = 7;
                break;
            case 2:
                x = 5;
                y = 7;
                break;
            case 3:
                x = 8;
                y = 7;
                break;
            case 4:
                x = 2;
                y = 4;
                break;
            case 5:
                x = 5;
                y = 4;
                break;
            case 6:
                x = 8;
                y = 4;
                break;
            case 7:
                x = 2;
                y = 1;
                break;
            case 8:
                x = 5;
                y = 1;
                break;
            case 9:
                x = 8;
                y = 1;
                break;
        
            default:
                break;
        }
        Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(180));

        return targetPose;
    }
}
