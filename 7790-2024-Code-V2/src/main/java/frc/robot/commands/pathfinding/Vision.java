package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Vision {

    public static Pose2d getPose(){
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-shooter");
        
        
            final double[] value = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            long m_tid = table.getEntry("tid").getInteger(0L);
            double m_x = value[0];
            double m_y = value[1];
            double m_z = value[2];
            double m_roll = value[3] * 3.141592653589793 / 180.0;
            double m_pitch = value[4] * 3.141592653589793 / 180.0;
            double m_yaw = value[5] * 3.141592653589793 / 180.0;
            SmartDashboard.putNumber("X Value", value[0]);
            SmartDashboard.putNumber("Y Value", value[1]);
            SmartDashboard.putNumber("Z Value", value[2]);
            SmartDashboard.putNumber("Roll Value", value[3]);
            SmartDashboard.putNumber("pitch Value", value[4]);
            SmartDashboard.putNumber("Yaw Value", value[5]);
            SmartDashboard.putNumber("target", (double)m_tid);

            Pose2d pose = new Pose2d(m_x,m_y,new Rotation2d(m_yaw));

            if(m_tid < 1 || m_tid > 16)
            {
                return null;
            }
            {
                return pose;
            }
           
           
        }
 
}
