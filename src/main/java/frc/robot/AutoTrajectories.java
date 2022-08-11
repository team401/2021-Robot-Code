package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class AutoTrajectories {

    /**
     * Defines the trajectories to be run through auto
     * 
     * Full field length: 630 inches
     * Full field width: 324 inches
     */

    // configures the maximum velocity and accel for the trajectories
    private final static TrajectoryConfig config = 
        new TrajectoryConfig(
            AutoConstants.maxVelMetersPerSec, 
            AutoConstants.maxAccelMetersPerSecondSq
        )
        .setKinematics(DriveConstants.kinematics);
    
    // start centered on line, center 4 ft from right edge
    public static Trajectory startRightToTrenchRight = 
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(Units.inchesToMeters(510), Units.inchesToMeters(48), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(423), Units.inchesToMeters(27.75), new Rotation2d(Math.PI)),
                new Pose2d(Units.inchesToMeters(310), Units.inchesToMeters(27.75), new Rotation2d(Math.PI))
            ), 
            config
        );

    public static Trajectory trenchRightToShootRight = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(310), Units.inchesToMeters(27.75), new Rotation2d(Math.PI)),
            List.of(
                new Translation2d(Units.inchesToMeters(423), Units.inchesToMeters(27.75))
            ),
            new Pose2d(Units.inchesToMeters(510), Units.inchesToMeters(48), new Rotation2d(0)), // TODO: update angle
            config
        );

    // start centered on line, center 13 ft from right edge
    public static Trajectory startMidToTrenchRight = 
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(Units.inchesToMeters(510), Units.inchesToMeters(162), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(423), Units.inchesToMeters(27.75), new Rotation2d(Math.PI)),
                new Pose2d(Units.inchesToMeters(310), Units.inchesToMeters(27.75), new Rotation2d(Math.PI))
            ), 
            config
        );

    public static Trajectory startRightToGenerator = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(510), Units.inchesToMeters(48), new Rotation2d(0)),
            List.of(
                new Translation2d(Units.inchesToMeters(315), Units.inchesToMeters(85)),
                new Translation2d(Units.inchesToMeters(373), Units.inchesToMeters(132)),
                new Translation2d(Units.inchesToMeters(390.5), Units.inchesToMeters(144)),
                new Translation2d(Units.inchesToMeters(378), Units.inchesToMeters(100))
            ), 
            new Pose2d(Units.inchesToMeters(510), Units.inchesToMeters(65), new Rotation2d(0)),
            config
        );

    // start centered on line, center 4 ft from left edge
    /*public static Trajectory startLeftToTrenchLeft = 
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(Units.inchesToMeters(510), Units.inchesToMeters(320), new Rotation2d(0))
            ), 
            config
        );*/

    public static Trajectory startRightDriveOffInitLine = 
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(Units.inchesToMeters(510), Units.inchesToMeters(48), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(450), Units.inchesToMeters(48), new Rotation2d(0))
            ), 
            config
        ); 

    public static Trajectory startMidDriveOffInitLine = 
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(Units.inchesToMeters(510), Units.inchesToMeters(162), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(450), Units.inchesToMeters(162), new Rotation2d(0))
            ), 
            config
        ); 

    public static Trajectory startLeftDriveOffInitLine = 
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(Units.inchesToMeters(510), Units.inchesToMeters(320), new Rotation2d(0)),
                new Pose2d(Units.inchesToMeters(450), Units.inchesToMeters(320), new Rotation2d(0))
            ), 
            config
        ); 

}