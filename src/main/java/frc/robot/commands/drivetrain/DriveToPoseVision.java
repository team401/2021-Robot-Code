package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class DriveToPoseVision extends CommandBase {

    private final DriveSubsystem drive;
    private final Limelight limelight;

    private final Pose2d desiredPose;

    private final ProfiledPIDController xController = 
        new ProfiledPIDController(
        1, 0, 0, 
            new TrapezoidProfile.Constraints(Units.inchesToMeters(60), Units.inchesToMeters(24))
        );

    private final ProfiledPIDController yController = 
        new ProfiledPIDController(
            1, 0, 0, 
            new TrapezoidProfile.Constraints(
                Units.inchesToMeters(12), 
                Units.inchesToMeters(12)
            )
        );

    private final ProfiledPIDController rotationController = 
        new ProfiledPIDController(
            1, 0, 0, 
            new TrapezoidProfile.Constraints(
                Units.inchesToMeters(12), 
                Units.inchesToMeters(12)
            )
        );

    public DriveToPoseVision(Pose2d pose, DriveSubsystem subsystem, Limelight vision) {

        desiredPose = pose;

        drive = subsystem;
        limelight = vision;

        addRequirements(drive, limelight);

        xController.setTolerance(Units.inchesToMeters(1));
        yController.setTolerance(Units.inchesToMeters(1));
        rotationController.setTolerance(Units.degreesToRadians(2));

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void execute() {

        boolean hasLimelightLock = limelight.hasValidTarget();

        double yOut = xController.calculate(drive.getPose().getY(), desiredPose.getX());
        double xOut = yController.calculate(drive.getPose().getX(), desiredPose.getY());

        double rotationOut = 
            (hasLimelightLock)
            ? rotationController.calculate(limelight.gettX(), 0) 
            : rotationController.calculate(
                drive.getPose().getRotation().getRadians(),
                desiredPose.getRotation().getRadians()
            );

        drive.drive(xOut, yOut, rotationOut, false);  // strafe, forward, rotation

    }

    @Override
    public boolean isFinished() {

        return (
            xController.atGoal() 
            && yController.atGoal()
            && (rotationController.atGoal() || limelight.hasValidTarget())
        );

    }
    
}
