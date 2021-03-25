package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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
            List.of(
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(48.5), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(150), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(48), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(60), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(90), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(90), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(250), Units.inchesToMeters(135), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(210), Units.inchesToMeters(110), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(80), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(295), Units.inchesToMeters(30), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(295), Units.inchesToMeters(80), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(100), Units.inchesToMeters(80), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(48.5), Units.inchesToMeters(80), new Rotation2d(0))
            ), 
            config
        );

}