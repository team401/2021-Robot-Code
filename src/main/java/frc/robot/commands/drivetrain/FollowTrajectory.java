package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.trajectory.Trajectory;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends SwerveControllerCommand {

    private final Trajectory trajectory;

    private static final ProfiledPIDController rotationController = 
        new ProfiledPIDController(2, 0, 0,
            new TrapezoidProfile.Constraints(AutoConstants.maxVelMetersPerSec,
                AutoConstants.maxAccelMetersPerSecondSq
            )
        );

    public FollowTrajectory(DriveSubsystem drive, Trajectory trajectory) {

        super(
            trajectory, 
            drive::getPose, 
            DriveConstants.kinematics, 
            new PIDController(1.5, 0, 0),
            new PIDController(1.5, 0, 0), 
            rotationController,
            drive::setModuleStates, 
            drive
        );

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        this.trajectory = trajectory;
    
    }

    public Pose2d getInitialPose() {
        
        return trajectory.getInitialPose();

    }

}