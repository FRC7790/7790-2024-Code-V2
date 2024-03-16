package frc.robot.commands.pathfinding;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LinearSystem {
    public static ArrayList<Pose2d> poses;

    static Boolean isInitialized = false;

    public static void initialize ()
    {
        if(!isInitialized)
        {
            poses = new ArrayList<Pose2d>();
            poses.add(new Pose2d(1.9,-28.0,Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(2.2,-25.4,Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(2.5,-20.2,Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(3.0,-15.7,Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(3.28,-14.0,Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(3.4,-16.0,Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(3.55,-12.65,Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(3.78,-10.93,Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(3.99,-14.55,Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(4.01,-12.8,Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(4.13,-9.1,Rotation2d.fromDegrees(0)));
            poses.add(new Pose2d(5.4,-7.0,Rotation2d.fromDegrees(0)));
            isInitialized = true;
        }
    }

    public static double getAngle(double distance){

        initialize();
        double angle = poses.get(0).getY();

        Pose2d closestBelow = poses.get(0);
        Pose2d closestAbove = poses.get(1);

       
        for (int i = 1 ; i < poses.size() - 1; i++){
            
            double min = poses.get(i).getX();
            double max = poses.get(i+1).getX();

            if(distance <= max && distance >= min)
            {
                closestBelow = poses.get(i);
                closestAbove = poses.get(i+1);
            }
        }

        double range = closestAbove.getX() - closestBelow.getX();
        double d = (distance - closestBelow.getX())/range;
        angle = MathUtil.interpolate(closestBelow.getY(), closestAbove.getY(), d);

        //System.out.println(angle);

        
        return angle;
    }
}
