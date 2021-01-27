package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class OperatorControl extends CommandBase {

    private final DriveSubsystem drive;

    private final DoubleSupplier forwardX;
    private final DoubleSupplier forwardY;
    private final DoubleSupplier rotation;

    public OperatorControl(final DriveSubsystem subsystem, final DoubleSupplier fwdX, final DoubleSupplier fwdY, final DoubleSupplier rot) {

        drive = subsystem;
        forwardX = fwdX;
        forwardY = fwdY;
        rotation = rot;
        addRequirements(subsystem);

    }  
    
    @Override
    public void execute() {

        drive.drive(forwardX.getAsDouble(), forwardY.getAsDouble(), rotation.getAsDouble(), false);

    }

}