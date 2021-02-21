package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRetract extends CommandBase {

    private final IntakeSubsystem subsystem;

    public IntakeRetract(IntakeSubsystem Intake) {

        subsystem = Intake;
        addRequirements(subsystem);

    }

    public void initialize() {

        subsystem.retractIntake();

    }

    public boolean isFinished() {

        return true;
        
    }
}