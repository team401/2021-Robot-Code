package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignWithTargetVision extends CommandBase {

    /**
    * Similar to QuickTurn, a command to control robot heading based on a vision target
    */

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

        //turn on the limelight at the beginning of the command
        limelight.setLedMode(0);

    }

    @Override
    public void execute() {

        /**
         * Runs only if the limelight has a lock on a valid target
         * Calculate the desired output and controls the rotational output of the drive to lock to the target
         * Allows for manual strafing through joystick input
         */
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

        //turn off the limelight led when the command ends (button is released)
        limelight.setLedMode(1);

    }

}
