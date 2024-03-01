package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ButtonMapping {
    public static Pose2d buttonToPose(int buttonNum){
        float x = 0;
        float y = 0;

        switch (buttonNum) {
            case 1:
                x = 5;
                y = 4;
                break;
            case 2:
                x = 2.5f;
                y = 2.5f;
                break;
            case 3:
                x = 3;
                y = 6.5f;
                break;
            case 4:
                x = 8.3f;
                y = 6.7f;
                break;
            case 5:
                x = 8.3f;
                y = 4.1f;
                break;
            case 6:
                x = 8.3f;
                y = 1.6f;
                break;
            case 7:
                x = 13.5f;
                y = 1.5f;
                break;
            /* case 8:
                x = 5;
                y = 1;
                break;
            case 9:
                x = 8;
                y = 1;
                break; */
        
            default:
                break;
        }
        Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(180));

        return targetPose;
    }
}
