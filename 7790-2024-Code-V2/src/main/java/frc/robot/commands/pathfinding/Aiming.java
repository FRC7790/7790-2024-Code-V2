package frc.robot.commands.pathfinding;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;



public class Aiming {

    public static boolean isAiming = false;

    public static float angle = -27;

    public static void setIsAiming() {
        isAiming = true;
    }

    public static void setIsNotAiming() {
        isAiming = false;
    }

    public Command setIsAimingCommand()
    {
        Command command = new InstantCommand(()-> Aiming.setIsAiming());
        return command;
    }

    public Command setIsNotAimingCommand()
    {
        Command command = new InstantCommand(()-> Aiming.setIsNotAiming());
        return command;
    }

    public static Translation2d getDiffPoseToScore(Pose2d botPose)
    {
        // SpeakerPose
        Pose2d speakerPose = new Pose2d (0.0,5.55, Rotation2d.fromDegrees(0));

        var alliance = DriverStation.getAlliance();
         boolean isRedAlliance =  alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;

         if(isRedAlliance){

            speakerPose = new Pose2d (16.54,5.55, Rotation2d.fromDegrees(0));
         }

        Translation2d diffPose = botPose.minus(speakerPose).getTranslation();

        return diffPose;
        
    }

    public static double getShootAngle(Pose2d botPose) {
        
        Pose2d speakerPose = new Pose2d (0.0,5.55, Rotation2d.fromDegrees(0));

        var alliance = DriverStation.getAlliance();
         boolean isRedAlliance =  alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;

         if(isRedAlliance){

            speakerPose = new Pose2d (16.54,5.55, Rotation2d.fromDegrees(0));
         }

        Translation2d diffPose = botPose.minus(speakerPose).getTranslation();
         
        double dist = diffPose.getNorm();
        
        //System.out.println(dist);

        angle = 0;

        if (dist < 1.9) {
            angle = -31;
        }
        else if (1.9 < dist && dist < 2.2) {
            angle = -28;
        }
        else if (2.2 < dist && dist < 2.5) {
            angle = -26;
        }
        else if (2.5 < dist && dist < 2.9) {
            angle = -22;
        }
        else if (2.9 < dist && dist < 3.4) {
            angle = -19;
        }
        else {
            angle = -17;
        }
        return angle;
}








 /*    public static double getAngleOfLineBetweenTwoPoints(Pose2d aimingPose) {

        
        
        double computedAngle = 0;

        double fieldWidth = 16.54;
        double midField = fieldWidth / 2;

        float x1;
        float y1;
        float x2;
        float y2;
        float xdis;
        float ydis;

        // Set to AimingPose
        x1 = 0;
        y1 = 0;

        // SpeakerPose
        x2 = 0;
        y2 = 5.55f;

        System.out.println(aimingPose);
        

        var alliance = DriverStation.getAlliance();
        boolean isRedAlliance = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;

        if (isRedAlliance) {

            x1 = (float) (fieldWidth - x1);

            x2 = (float) (fieldWidth - x2);
        }

        



        double xDiff = x1 - x2;
        double yDiff = y1 - y2;

        computedAngle = Math.toDegrees(Math.atan2(yDiff, xDiff));

        return computedAngle;
    } */
}
