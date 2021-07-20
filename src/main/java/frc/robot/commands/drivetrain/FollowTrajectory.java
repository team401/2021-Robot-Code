package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.AutoTrajectories;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends SwerveControllerCommand {

    /**
     * Command to follow a given Trajectory using the SwerveControllerCommand class, which in turn uses HolonomicDriveController
     */

    private final Trajectory trajectory;
    private Timer timer = new Timer();
    private DriveSubsystem drive;

    //uses motion profiling versus standard PID for smoother heading tracking and to limit rotational speed
    private static final ProfiledPIDController rotationController = 
        new ProfiledPIDController(3.0, 0, 0,
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
            new PIDController(2.45, 0, 1.0),
            new PIDController(2.45, 0, 1.0), 
            rotationController,
            drive::setModuleStates, 
            drive
        );

        this.drive = drive;

        // set the rotation controller to wrap around from -PI to PI
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        this.trajectory = trajectory;
    
    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

        super.initialize();

    } 

    @Override
    public void execute() {

        super.execute();

    }

    public Pose2d getInitialPose() {
        
        return trajectory.getInitialPose();

    }

}