package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStop extends CommandBase {

    private final IntakeSubsystem subsystem;

    public IntakeStop(IntakeSubsystem intake) {

        addRequirements(intake);
        subsystem = intake;

    }

    public void initialize() {

        subsystem.stopIntakeMotor();

    }

    public boolean isFinished() {

        return true;

    }
    
}
