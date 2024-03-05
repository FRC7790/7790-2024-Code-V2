package frc.robot.commands.pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;



public class Aiming {

    public static boolean isAiming = false;

    public static void setIsAiming() {
        boolean isAiming = true;
    }

    public static void setIsNotAiming() {
        boolean isAiming = false;
    }

    public static double getAngleOfLineBetweenTwoPoints(Pose2d aimingPose) {

        
        
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
    }
}
