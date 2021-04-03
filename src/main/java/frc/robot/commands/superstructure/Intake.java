package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends CommandBase {
    
    private final IntakeSubsystem intake;

    public Intake(IntakeSubsystem subsystem) {

        intake = subsystem;

    }
    
    @Override
    public void initialize() {

        intake.extendIntake();

    }

    @Override
    public void execute() {

        intake.runIntakeMotor();

    }

    @Override
    public void end(boolean interrupted) {

        intake.retractIntake();
        intake.stopIntakeMotor();
 
    }

}
