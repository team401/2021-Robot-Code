package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignWithTargetVision extends CommandBase {

    /**
    * Similar to QuickTurn, a command to control robot heading based on a vision target
    */

    private final DriveSubsystem drive;
    private final VisionSubsystem limelight;

    private double rotationOut = 0;

    private final PIDController controller = new PIDController(
        5.0, 0, 0
    );

    public AlignWithTargetVision(DriveSubsystem subsystem, VisionSubsystem vision) {

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

        SmartDashboard.putNumber("tx value", Units.radiansToDegrees(limelight.gettX()));

        /**
         * Runs only if the limelight has a lock on a valid target
         * Calculate the desired output and controls the rotational output of the drive to lock to the target
         * Allows for manual strafing through joystick input
         */
        if (limelight.hasValidTarget()) {

            if (Math.abs(limelight.gettX()) > Units.degreesToRadians(1.5)) rotationOut = controller.calculate(limelight.gettX(), Units.degreesToRadians(0)); 
            else rotationOut = 0;

            drive.drive(
                drive.getCommandedDriveValues()[0], 
                drive.getCommandedDriveValues()[1], 
                rotationOut, 
                drive.getIsFieldRelative());

            SmartDashboard.putNumber("rotation out", rotationOut);

        }

    }

    @Override
    public boolean isFinished() {

        if (limelight.hasValidTarget()) return Math.abs(limelight.gettX()) < Units.degreesToRadians(1.5);
        else return false;

    }

    @Override
    public void end(boolean interrupted) {
    
        SmartDashboard.putString("is finished", "yes");

        //turn off the limelight led when the command ends (button is released)
        limelight.setLedMode(1);
    

    }

}
