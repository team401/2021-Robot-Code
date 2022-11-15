package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class OperatorControl extends CommandBase {

    /**
     * Command to allow for driver input in teleop
     * Can't be inlined efficiently if we want to edit the inputs in any way (deadband, square, etc.)
     */

    private final DriveSubsystem drive;

    /**
     * Joysticks return DoubleSuppliers when the get methods are called
     * This is so that joystick getter methods can be passed in as a parameter but will continuously update, 
     * versus using a double which would only update when the constructor is called
     */
    private final DoubleSupplier forwardX;
    private final DoubleSupplier forwardY;
    private final DoubleSupplier rotation;
    
    private final boolean isFieldRelative;

    public OperatorControl(
        DriveSubsystem subsystem, 
        DoubleSupplier fwdX, 
        DoubleSupplier fwdY, 
        DoubleSupplier rot,
        boolean fieldRelative
    ) {

        drive = subsystem;
        forwardX = fwdX;
        forwardY = fwdY;
        rotation = rot;

        isFieldRelative = fieldRelative;

        addRequirements(subsystem);

    }
    
    @Override
    public void execute() {

        /**
         * Units are given in meters per second radians per second
         * Since joysticks give output from -1 to 1, we multiply the outputs by the max speed
         * Otherwise, our max speed would be 1 meter per second and 1 radian per second
         */

        double driveSpeed = SmartDashboard.getNumber("Max Drive Speed", DriveConstants.maxDriveSpeed);
        double turnRateDegPerSec = SmartDashboard.getNumber("Turn Rate Deg/s", DriveConstants.teleopTurnRateDegPerSec);

        double fwdX = forwardX.getAsDouble();
        fwdX = Math.copySign(fwdX, fwdX);
        fwdX = deadbandInputs(fwdX) * Units.feetToMeters(driveSpeed);

        double fwdY = forwardY.getAsDouble();
        fwdY = Math.copySign(fwdY, fwdY);
        fwdY = deadbandInputs(fwdY) * Units.feetToMeters(driveSpeed);

        double rot = rotation.getAsDouble();
        rot = Math.copySign(rot * rot, rot);
        rot = deadbandInputs(rot) * Units.degreesToRadians(turnRateDegPerSec);

        drive.drive(
            -fwdX,
            -fwdY,
            -rot,
            isFieldRelative
        );

    }

    // method to deadband inputs to eliminate tiny unwanted values from the joysticks
    public double deadbandInputs(double input) {

        if (Math.abs(input) < 0.1) return 0.0;
        return input;

    }

    @Override
    public void end(boolean interrupted) {

        drive.drive(0, 0, 0, true);

    }

}