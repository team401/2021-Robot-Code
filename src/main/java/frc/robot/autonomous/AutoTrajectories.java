package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class AutoTrajectories { // coordinates are cartesian

    private final static TrajectoryConfig config = 
        new TrajectoryConfig(
            AutoConstants.maxVelMetersPerSec, 
            AutoConstants.maxAccelMetersPerSecondSq
        )
        .setKinematics(DriveConstants.kinematics);

    public static Trajectory autoNavBarrelTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(1, 1, new Rotation2d(0))

            //new Pose2d(4, 1.15, new Rotation2d(0)),
            //new Pose2d(2.5, 3, new Rotation2d(0)),
            //new Pose2d(2.5, 5, new Rotation2d(0))
            ), 
            config
        );

}