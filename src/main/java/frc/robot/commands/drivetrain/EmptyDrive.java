package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class EmptyDrive extends CommandBase {

    public EmptyDrive(DriveSubsystem drive) {

        addRequirements(drive);

    }
    
}
