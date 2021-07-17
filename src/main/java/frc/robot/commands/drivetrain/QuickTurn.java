package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class QuickTurn extends CommandBase {

    /**
     * A command to align the robot automatically to a certain heading, used for a "quick turn" for shooting
     */

    private final DriveSubsystem drive;

    private final double desiredAngle;

    private final PIDController controller = new PIDController(
        5.0, 0, 0
    );

    public QuickTurn(DriveSubsystem subsystem, double desiredAngleRad) {

        drive = subsystem;

        desiredAngle = desiredAngleRad;

    }

    @Override
    public void execute() {
            
        //calculate the next output for the drive based on the current heading, setpoint being the given angle

        if (Math.abs(desiredAngle - drive.getHeading().getRadians()) > Units.degreesToRadians(1)) {

            double rotationOut = controller.calculate(drive.getHeading().getRadians(), desiredAngle);

            drive.drive(
                drive.getCommandedDriveValues()[0], 
                drive.getCommandedDriveValues()[1], 
                rotationOut, 
                drive.getIsFieldRelative());

        }

    }

}
