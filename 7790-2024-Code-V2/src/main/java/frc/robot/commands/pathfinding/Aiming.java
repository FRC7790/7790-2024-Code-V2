package frc.robot.commands.pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;



public class Aiming {

    public static boolean isAiming = false;

    public static void setIsAiming() {
        isAiming = true;
    }

    public static void setIsNotAiming() {
        isAiming = false;
    }

    public static Translation2d getStickPoseToScore(Pose2d botPose)
    {
        // SpeakerPose
        Pose2d speakerPose = new Pose2d (0.0,5.55, Rotation2d.fromDegrees(0));

        Translation2d diffPose = botPose.minus(speakerPose).getTranslation();

        return diffPose;
        
    }



// flip for red alliance add








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
