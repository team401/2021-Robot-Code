package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.Ultrasonic.Unit;
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
            new Pose2d(Units.inchesToMeters(48.5), Units.inchesToMeters(30), new Rotation2d(0)), 
            List.of(
                new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(90)),    
                new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(60)),     
                new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(280), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(20)),
                new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(20)),
                new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(55))
            ),
            new Pose2d(Units.inchesToMeters(48.5), Units.inchesToMeters(75), new Rotation2d(0)), 
            config
        );

            //preliminary pathing for barrel
    public static Trajectory autoNavBarrelTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(48.5), Units.inchesToMeters(90), new Rotation2d(0)),
            List.of(
                new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(48)),
                new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(250), Units.inchesToMeters(135)),
                new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(110)),
                new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(80)),
                new Translation2d(Units.inchesToMeters(295), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(295), Units.inchesToMeters(80)),
                new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(80))
            ), 
            new Pose2d(Units.inchesToMeters(48.5), Units.inchesToMeters(80), new Rotation2d(0)),
            config
        );

}