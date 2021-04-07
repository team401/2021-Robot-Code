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

    public static Trajectory autoNavSlalomTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(30), new Rotation2d(0)),
            List.of(
                new Translation2d(Units.inchesToMeters(59), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(83), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(260), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(288), Units.inchesToMeters(20)),
                new Translation2d(Units.inchesToMeters(323), Units.inchesToMeters(29)),
                new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(323), Units.inchesToMeters(87)),
                new Translation2d(Units.inchesToMeters(290), Units.inchesToMeters(95)),
                new Translation2d(Units.inchesToMeters(265), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(59), Units.inchesToMeters(100))
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

    public static Trajectory autoNavBarrelTrajectory =
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(90), new Rotation2d(0)),
        List.of(
            new Translation2d(Units.inchesToMeters(158), Units.inchesToMeters(78)),
            new Translation2d(Units.inchesToMeters(170), Units.inchesToMeters(60)),
            new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(30)),
            new Translation2d(Units.inchesToMeters(115), Units.inchesToMeters(70)),
            new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(90)),
            new Translation2d(Units.inchesToMeters(262), Units.inchesToMeters(106)),
            new Translation2d(Units.inchesToMeters(275), Units.inchesToMeters(126)),
            new Translation2d(Units.inchesToMeters(248), Units.inchesToMeters(165)),
            new Translation2d(Units.inchesToMeters(195), Units.inchesToMeters(121)),
            new Translation2d(Units.inchesToMeters(280), Units.inchesToMeters(20)),
            new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(20)),
            new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(80))
        ),  
        new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(80), new Rotation2d(0)),
        config
    );
}