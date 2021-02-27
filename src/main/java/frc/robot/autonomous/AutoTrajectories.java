package frc.robot.autonomous;

import java.util.List;

import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.PathArcSegment;
import org.frcteam2910.common.control.SplinePathGenerator;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.spline.*;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class AutoTrajectories {

    private final static TrajectoryConfig config = 
        new TrajectoryConfig(
            AutoConstants.maxVelMetersPerSec, 
            AutoConstants.maxAccelMetersPerSecondSq
        )
        .setKinematics(DriveConstants.kinematics);

    public static Trajectory testTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1, 1)
            ),
            new Pose2d(5, 0, new Rotation2d(Math.PI)),
            config
        );
    
    public static Trajectory firstBarrel = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), 
                List.of(
                    new Translation2d(1.5, -1.0)
                ), 
                new Pose2d(3.24, -2.25, new Rotation2d(Math.PI)), 
                config);
    
}