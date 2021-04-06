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

    public static Trajectory autoNavBarrelTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(30), new Rotation2d(0)),
            List.of(
                new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(60))
            ), 
            new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(0)),
            config
        );

    public static Trajectory autoNavBounceTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(160), new Rotation2d(0)), 
            List.of(
                new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(160)),
                new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(250), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(160))
            ), 
            new Pose2d(Units.inchesToMeters(330), Units.inchesToMeters(90), new Rotation2d(0)), 
            config
        );

}