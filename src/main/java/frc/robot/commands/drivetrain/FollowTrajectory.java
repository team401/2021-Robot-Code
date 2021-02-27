package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends SwerveControllerCommand {

    private static final ProfiledPIDController rotationController = 
        new ProfiledPIDController(0.5, 0, 0,
            new TrapezoidProfile.Constraints(AutoConstants.maxVelMetersPerSec,
                AutoConstants.maxAccelMetersPerSecondSq
            )
        );

    static {

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public FollowTrajectory(DriveSubsystem drive, Trajectory trajectory) {

        super(
            trajectory, 
            drive::getPose, 
            DriveConstants.kinematics, 
            new PIDController(0.5, 0, 0.0001),
            new PIDController(0.5, 0, 0.0001), 
            rotationController,
            drive::setModuleStates, 
            drive
        );
    
    }
    
}


        