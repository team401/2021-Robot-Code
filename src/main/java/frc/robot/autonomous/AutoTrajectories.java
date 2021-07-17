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

    /**
     * Defines the trajectories to be run through auto
     */

    // configures the maximum velocity and accel for the trajectories
    private final static TrajectoryConfig config = 
        new TrajectoryConfig(
            AutoConstants.maxVelMetersPerSec, 
            AutoConstants.maxAccelMetersPerSecondSq
        )
        .setKinematics(DriveConstants.kinematics);

    /**
     * Each trajectory is treated as a constant statically through the class
     * 
     * Trajectories are cubic clamped - generated with automatically defined slopes
     */
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
            new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(90), new Rotation2d(0)), 
            List.of(
                new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(85), Units.inchesToMeters(150)),
                new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(75)),
                new Translation2d(Units.inchesToMeters(165), Units.inchesToMeters(75)),
                new Translation2d(Units.inchesToMeters(170), Units.inchesToMeters(215)),
                new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(130)),
                new Translation2d(Units.inchesToMeters(260), Units.inchesToMeters(130)),
                new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(275)),
                new Translation2d(Units.inchesToMeters(280), Units.inchesToMeters(240))
            ),
            new Pose2d(Units.inchesToMeters(320), Units.inchesToMeters(240), new Rotation2d(0)), 
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

    public static Trajectory galacticSearchRedATrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(96), new Rotation2d(0)),
            List.of(    
                new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(140))
            ),
            new Pose2d(Units.inchesToMeters(370), Units.inchesToMeters(140), new Rotation2d(0)),
            config
        );

    public static Trajectory galacticSearchRedBTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(96), new Rotation2d(0)),
            List.of(
                new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(120)),
                new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(120))
            ),
            new Pose2d(Units.inchesToMeters(370), Units.inchesToMeters(120), new Rotation2d(0)),
            config
    );

    public static Trajectory testTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
            List.of(
                new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(15)),
                new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(0))
            ), 
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(30), new Rotation2d(Math.PI)),
            config
    );

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