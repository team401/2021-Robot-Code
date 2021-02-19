package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeCommand(IntakeSubsystem subsystem) {
        intakeSubsystem = subsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.runIntakeMotor();
    }

    @Override
    public void initialize() {
        intakeSubsystem.extendIntake();
    }

    @Override
    public void end() {
        intakeSubsystem.retractIntake();
        intakeSubsystem.stopIntakeMotor();
    }

    @Override
    public boolean isFinished(Boolean condition) {
        return true;
    }

}
