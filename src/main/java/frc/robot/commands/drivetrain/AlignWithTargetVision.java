package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignWithTargetVision extends CommandBase {

    private final DriveSubsystem drive;
    private final Limelight limelight;

    private final PIDController controller = new PIDController(
        2.5, 0, 0
    );

    public AlignWithTargetVision(DriveSubsystem subsystem, Limelight vision) {

        drive = subsystem;
        limelight = vision;

    }

    public void execute() {

        if (limelight.hasValidTarget()) {
            
            double rotationOut = controller.calculate(limelight.gettX(), 0);

            drive.drive(
                drive.getCommandedDriveValues()[0], 
                drive.getCommandedDriveValues()[1], 
                rotationOut, 
                drive.getIsFieldRelative());

        }

    }

}
