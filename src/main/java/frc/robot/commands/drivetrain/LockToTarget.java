package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class LockToTarget extends CommandBase {

    private final DriveSubsystem drive;

    private final Pose2d robotToTarget;
    
    public LockToTarget(DriveSubsystem subsystem, Translation2d initialRobotToTargetTranslation, Rotation2d initialRobotToTargetRotation) {

        drive = subsystem;

        robotToTarget = new Pose2d(initialRobotToTargetTranslation, initialRobotToTargetRotation);

        addRequirements(drive);

    }

    @Override
    public void execute() {

        

    }

}
