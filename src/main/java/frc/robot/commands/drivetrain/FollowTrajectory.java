package frc.robot.commands.drivetrain;

import java.util.List;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory {

    private final DriveSubsystem drive = new DriveSubsystem();
    
    TrajectoryConfig config = 
        new TrajectoryConfig(
            AutoConstants.maxVelMetersPerSec,
            AutoConstants.maxAccelMetersPerSecondSq)
        .setKinematics(DriveConstants.kinematics);

    Trajectory trajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(1, 1), new Translation2d(2, -1)), 
        new Pose2d(3, 0, new Rotation2d(0)), 
        config);

    ProfiledPIDController rotationController = 
        new ProfiledPIDController(1, 0, 0, 
            new TrapezoidProfile.Constraints(
                AutoConstants.maxVelMetersPerSec, 
                AutoConstants.maxAccelMetersPerSecondSq));

SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            drive::getPose,
            DriveConstants.kinematics,
            new PIDController(1, 0.0, 0.0),
            new PIDController(1, 0.0, 0.0),
            rotationController,
            drive::setModuleStates,
            drive);

    public FollowTrajectory() {

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public SwerveControllerCommand runCommand() {
        
        return swerveControllerCommand;

    }
    
}
