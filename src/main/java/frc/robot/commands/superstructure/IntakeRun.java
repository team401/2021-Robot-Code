package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRun extends CommandBase{

    private final IntakeSubsystem subsystem;

    public IntakeRun(IntakeSubsystem intake){
        addRequirements(intake);
        subsystem = intake;
    }

    public void execute() {
        subsystem.runIntakeMotor();
    }

    public boolean isFinished() {
        return true;
    }
}
