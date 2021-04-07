package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class OperatorControl extends CommandBase {

    private final DriveSubsystem drive;

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

        double fwdX = forwardX.getAsDouble();
        fwdX = Math.copySign(fwdX, fwdX);
        fwdX = deadbandInputs(fwdX) * Units.feetToMeters(DriveConstants.maxDriveSpeed);

        double fwdY = forwardY.getAsDouble();
        fwdY = Math.copySign(fwdY, fwdY);
        fwdY = deadbandInputs(fwdY) * Units.feetToMeters(DriveConstants.maxDriveSpeed);

        double rot = rotation.getAsDouble();
        rot = Math.copySign(rot * rot, rot);
        rot = deadbandInputs(rot) * Units.degreesToRadians(DriveConstants.teleopTurnRateDegPerSec);

        drive.drive(
            -fwdX,
            -fwdY,
            -rot,
            isFieldRelative
        );

    }

    public double deadbandInputs(double input) {

        if (Math.abs(input) < 0.035) return 0.0;
        return input;

    }

}