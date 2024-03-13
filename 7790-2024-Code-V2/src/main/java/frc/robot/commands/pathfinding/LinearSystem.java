package frc.robot.commands.pathfinding;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LinearSystem {
    public static ArrayList<Pose2d> poses;

    public LinearSystem ()
    {
        poses = new ArrayList<Pose2d>();
        poses.add(new Pose2d(1.9,-31.0,Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(2.2,-28.0,Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(2.5,-26.0,Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(2.9,-22.0,Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(3.4,-19.0,Rotation2d.fromDegrees(0)));
        poses.add(new Pose2d(5.4,-7.0,Rotation2d.fromDegrees(0)));
    }

    public static double getAngle(double distance){

        double angle = poses.get(0).getY();

        Pose2d closestBelow = poses.get(0);
        Pose2d closestAbove = poses.get(1);

       
        for (int i = 1 ; i < poses.size() - 1; i++){
            
            double min = poses.get(i).getX();
            double max = poses.get(i+1).getX();

            if(distance < max && distance > min)
            {
                closestBelow = poses.get(i);
                closestAbove = poses.get(i+1);
            }
        }

        double range = closestAbove.getX() - closestBelow.getX();
        double d = (distance - closestBelow.getX())/range;
        angle = MathUtil.interpolate(closestBelow.getY(), closestAbove.getY(), d);

        System.out.println(angle);
        return angle;
    }
}
