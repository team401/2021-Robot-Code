package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignWithTargetVision extends CommandBase {

    private final DriveSubsystem drive;
    private final Limelight limelight;

    private final PIDController controller = new PIDController(
        2.5, 0, 0.25
    );

    public AlignWithTargetVision(DriveSubsystem subsystem, Limelight vision) {

        drive = subsystem;
        limelight = vision;

    }

    @Override
    public void initialize() {

        limelight.setLedMode(0);

    }

    @Override
    public void execute() {

        if (limelight.hasValidTarget()) {
            
            double rotationOut = controller.calculate(limelight.gettX(), Units.degreesToRadians(0));

            drive.drive(
                drive.getCommandedDriveValues()[0], 
                drive.getCommandedDriveValues()[1], 
                rotationOut, 
                drive.getIsFieldRelative());

        }

    }

    @Override
    public void end(boolean interrupted) {

        limelight.setLedMode(1);

    }

}
