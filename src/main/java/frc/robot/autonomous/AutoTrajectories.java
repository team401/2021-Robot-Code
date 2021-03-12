
package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class AutoTrajectories {

    private static TrajectoryConfig config;

    public static Trajectory trajectory;

    public AutoTrajectories() {

       /* config = new TrajectoryConfig(
            AutoConstants.maxVelMetersPerSec, 
            AutoConstants.maxAccelMetersPerSecondSq
        )
        .setKinematics(DriveConstants.kinematics);

        config.setStartVelocity(0.0);
        config.setEndVelocity(0.0);

        String trajectoryJSON = "paths/AutoNavSlalomPath.wpilib.json";
    
        trajectory = new Trajectory();
    
        try {

            Path trajpath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            if (Files.exists(trajpath)){
                trajectory = TrajectoryUtil.fromPathweaverJson(trajpath);
                SmartDashboard.putString("yes", "yup file"); 
            } else {
                SmartDashboard.putString("none", "no file"); 
            }

        } catch(IOException ex) {

            //DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());

        }*/

    }

}