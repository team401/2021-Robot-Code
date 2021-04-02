package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AlignWithGyro extends CommandBase {

    private final DriveSubsystem drive;

    private final double desiredAngle;

    private final PIDController controller = new PIDController(
        2.5, 0, 0.25
    );

    public AlignWithGyro(DriveSubsystem subsystem, double desiredAngleRad) {

        drive = subsystem;

        desiredAngle = desiredAngleRad;

    }

    @Override
    public void execute() {
            
        double rotationOut = controller.calculate(drive.getHeading().getRadians(), desiredAngle);

        drive.drive(
            drive.getCommandedDriveValues()[0], 
            drive.getCommandedDriveValues()[1], 
            rotationOut, 
            drive.getIsFieldRelative());

    }

}
