package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class AutoTrajectories {

    private final static TrajectoryConfig config = 
        new TrajectoryConfig(
            AutoConstants.maxVelMetersPerSec, 
            AutoConstants.maxAccelMetersPerSecondSq
        )
        .setKinematics(DriveConstants.kinematics);

    public static Trajectory autoNavBarrelRacingTrajectory = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d(0)), 
                List.of(
                    new Translation2d(Units.feetToMeters(0), Units.feetToMeters(0))
                ), 
                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d(0)), 
                config
            );

    public static Trajectory autoNavSlalomTrajectory = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d(0)), 
                List.of(
                    new Translation2d(Units.feetToMeters(0), Units.feetToMeters(0))
                ), 
                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d(0)), 
                config
            );
    
    public static Trajectory autoNavBounceTrajectory = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d(0)), 
                List.of(
                    new Translation2d(Units.feetToMeters(0), Units.feetToMeters(0))
                ), 
                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d(0)), 
                config
            );

    public static Trajectory testTrajectory = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d(0)), 
                List.of(
                    new Translation2d(Units.feetToMeters(0), Units.feetToMeters(0))
                ), 
                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d(0)), 
                config
            );

}