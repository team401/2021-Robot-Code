package frc.robot;

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

    public static Trajectory testTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)), 
            List.of (
                new Translation2d(Units.inchesToMeters(1), Units.inchesToMeters(0))
            ), 
            new Pose2d(Units.inchesToMeters(2), Units.inchesToMeters(0), new Rotation2d(0)), 
            config
        );

    /**
     * Each trajectory is treated as a constant statically through the class
     * 
     * Trajectories are cubic clamped - generated with automatically defined slopes
     */
    
    /*public static Trajectory startLeftToTrenchLeft = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(507), Units.inchesToMeters(), new Rotation2d()), 
            List.of (
                new Translation2d(Units.inchesToMeters(), Units.inchesToMeters())
            ), 
            new Pose2d(Units.inchesToMeters(), Units.inchesToMeters(), new Rotation2d()), 
            config
        );
    
    public static Trajectory midToTrenchClose = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(), Units.inchesToMeters(), new Rotation2d()), 
            List.of (
                new Translation2d(Units.inchesToMeters(), Units.inchesToMeters())
            ), 
            new Pose2d(Units.inchesToMeters(), Units.inchesToMeters(), new Rotation2d()), 
            config
        );

    public static Trajectory rightToTrenchClose = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(), Units.inchesToMeters(), new Rotation2d()), 
            List.of (
                new Translation2d(Units.inchesToMeters(), Units.inchesToMeters())
            ), 
            new Pose2d(Units.inchesToMeters(), Units.inchesToMeters(), new Rotation2d()), 
            config
        );

    public static Trajectory trenchCloseToShoot = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(), Units.inchesToMeters(), new Rotation2d()), 
            List.of (
                new Translation2d(Units.inchesToMeters(), Units.inchesToMeters())
            ), 
            new Pose2d(Units.inchesToMeters(), Units.inchesToMeters(), new Rotation2d()), 
            config
        );
    
    public static Trajectory shootToTrenchFar = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(), Units.inchesToMeters(), new Rotation2d()), 
            List.of (
                new Translation2d(Units.inchesToMeters(), Units.inchesToMeters())
            ), 
            new Pose2d(Units.inchesToMeters(), Units.inchesToMeters(), new Rotation2d()), 
            config
        );

    public static Trajectory trenchFarToShoot = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(), Units.inchesToMeters(), new Rotation2d()), 
            List.of (
                new Translation2d(Units.inchesToMeters(), Units.inchesToMeters())
            ), 
            new Pose2d(Units.inchesToMeters(), Units.inchesToMeters(), new Rotation2d()), 
            config
        );

    */


    /*For competition - Being across from shooter (on right looking at it) and going backwards to collect
      Steps are to shoot 3x, and then turn around and go backwards, collecting the balls while moving
    public static Trajectory acrossFromShooterAndCollect = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(323.5 - 94.66)), 
            List.of (

            ), 
            end, 
            config
    );
    
    //For competition - Being across from shoot and going sideways to get out of way
    //Steps are to shoot 3x, and then turn around and go backwards at an angle 
    public static Trajectory acrossFromShooterAndCollect = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(323.5 - 94.66)), 
            List.of (

            ), 
            end, 
            config
    );

    //For competition - Starting at the middle, going to shoot, and then moving backwards to collect
    public static Trajectory acrossFromShooterAndCollect = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d( Units.inchesToMeters(120), Units.inchesToMeters(323.5/2.)), 
            List.of (

            ), 
            end, 
            config
    );

    //For competition - Starting at the middle, going to shoot, and then moving backwards at an angle to get out of way
    public static Trajectory acrossFromShooterAndCollect = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(323.5/2)), 
            List.of (

            ), 
            end, 
            config
    );

    //For competition - Starting at left (when looking at drivers), going to shoot, and then moving backwards to collect
    public static Trajectory acrossFromShooterAndCollect = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(94.66)), 
            List.of (

            ), 
            end, 
            config
    );

    //For competition - Starting at left (when looking at drivers), going to shoot, and then moving backwards at angle
        public static Trajectory acrossFromShooterAndCollect = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(94.66)), 
            List.of (

            ), 
            end, 
            config
    );

    */

}