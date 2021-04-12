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

    /**
     * Command to follow a given Trajectory using the SwerveControllerCommand class, which in turn uses HolonomicDriveController
     */

    private final Trajectory trajectory;

    //uses motion profiling versus standard PID for smoother heading tracking and to limit rotational speed
    private static final ProfiledPIDController rotationController = 
        new ProfiledPIDController(2, 0, 0,
            new TrapezoidProfile.Constraints(AutoConstants.maxVelMetersPerSec,
                AutoConstants.maxAccelMetersPerSecondSq
            )
        );

    public FollowTrajectory(DriveSubsystem drive, Trajectory trajectory) {

        /**
         * Super constructor for SwerveControllerCommand
         * Parameters: 
         * trajectory to be followed
         * method reference to the pose supplier
         * kinematics of the drive (wheel placements on robot)
         * x controller
         * y controller
         * rotation controller
         * method reference to the module control method
         * requirements (drive subsystem)
         */
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

        // set the rotation controller to wrap around from -PI to PI
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        this.trajectory = trajectory;
    
    }

    public Pose2d getInitialPose() {
        
        return trajectory.getInitialPose();

    }

}