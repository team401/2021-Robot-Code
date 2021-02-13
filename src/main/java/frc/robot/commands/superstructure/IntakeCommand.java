package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intake;

    private final ConveyorSubsystem conveyor;

    private boolean isCurrentBallInConveyor = false;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, ConveyorSubsystem conveyorSubsystem) {

        intake = intakeSubsystem;
        conveyor = conveyorSubsystem;

        addRequirements(intake, conveyor);

    }

    @Override
    public void initialize() {

        intake.extendIntake();

    }

    @Override
    public void execute() {

        if (conveyor.getBottomBannerState() && !conveyor.getTopBannerState()) {

            intake.runIntakeMotor();
            conveyor.runConveyor();

            Timer.delay(SuperstructureConstants.spacingDelaySeconds);

            isCurrentBallInConveyor = true;

        }

    }

    @Override
    public boolean isFinished() {

        return isCurrentBallInConveyor;
        
    }

    @Override
    public void end(boolean interrupted) {

        intake.retractIntake();

    }
    
}
