package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignWithTargetVision extends CommandBase {

    private final DriveSubsystem drive;
    private final Limelight limelight;

    private final ProfiledPIDController controller = new ProfiledPIDController(
        1.5, 0, 0, 
        new TrapezoidProfile.Constraints(
            0, 
            0  // actaully add values here
        )
    );

    public AlignWithTargetVision(DriveSubsystem subsystem, Limelight vision) {

        drive = subsystem;
        limelight = vision;

    }

    public void execute() {

        if (limelight.hasValidTarget()) {
            
            double rotationOut = controller.calculate(-limelight.gettX(), 0);

            drive.drive(
                drive.getCommandedDriveValues()[0], 
                drive.getCommandedDriveValues()[1], 
                rotationOut, 
                drive.getIsFieldRelative());

        }

    }

}
