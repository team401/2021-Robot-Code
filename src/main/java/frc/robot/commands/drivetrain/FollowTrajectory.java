package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.trajectory.Trajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends CommandBase {

    private final Trajectory trajectory;

    private Timer time = new Timer();

    private final SwerveDriveKinematics kinematics = DriveConstants.kinematics;

    private DriveSubsystem drive;

    private final Pose2d tolerance = 
        new Pose2d(
            Units.inchesToMeters(0.5), 
            Units.inchesToMeters(0.5), 
            new Rotation2d(0)
        );

    private static final ProfiledPIDController rotationController = 
        new ProfiledPIDController(2, 0, 0,
            new TrapezoidProfile.Constraints(
                AutoConstants.maxVelMetersPerSec,
                AutoConstants.maxAccelMetersPerSecondSq
            )
        );

    private static final HolonomicDriveController controller = 
        new HolonomicDriveController(
            new PIDController(1.1, 0, 0.2), 
            new PIDController(1.1, 0, 0.2), 
            rotationController
        );

    public FollowTrajectory(DriveSubsystem drives, Trajectory trajectory) {

        this.trajectory = trajectory;
        drive = drives;
        controller.setTolerance(tolerance);

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

    }
    
    public Pose2d getInitialPose() {

        return trajectory.getInitialPose();

    }

    public Pose2d getCurrentPose() {

        return drive.getPose();

    }

    @Override
    public void initialize() {

        time.reset();
        time.start();

    }

    @Override
    public void execute() {

        Trajectory.State goal = trajectory.sample(time.get());
        SmartDashboard.putString("X wanted", " " + goal.poseMeters.getX()); 
        SmartDashboard.putString("Y wanted", " " + goal.poseMeters.getY()); 
        ChassisSpeeds adjustedSpeeds = 
            controller.calculate(
                getCurrentPose(), 
                goal, 
                Rotation2d.fromDegrees(0.0)
            );
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(adjustedSpeeds);
        SmartDashboard.putString("Actual speed", " " + (drive.getModuleStates())[1].speedMetersPerSecond);
        SmartDashboard.putString("Speed wanted", " " + moduleStates[1].speedMetersPerSecond);     
        drive.setModuleStates(moduleStates);

    }

    @Override
    public void end(boolean interrupted) {

        time.stop();

    }

    @Override
    public boolean isFinished() {

        return time.hasElapsed(trajectory.getTotalTimeSeconds());

    }

}