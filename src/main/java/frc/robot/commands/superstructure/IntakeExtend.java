package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class IntakeExtend extends CommandBase {

    private final IntakeSubsystem subsystem;

    public IntakeExtend(IntakeSubsystem Intake) {

        subsystem = Intake;
        addRequirements(subsystem);

    }

    public void initialize() {

        subsystem.extendIntake();

    }

    public boolean isFinished() {

        return true;
        
    }
}
