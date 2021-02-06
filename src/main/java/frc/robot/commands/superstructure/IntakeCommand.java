package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.BallSubsystem;

public class IntakeCommand extends CommandBase {

    private final BallSubsystem ballSubsystem;

    public IntakeCommand(BallSubsystem subsystem) {

        ballSubsystem = subsystem;

        addRequirements(ballSubsystem);

    }

    @Override
    public void initialize() {

        ballSubsystem.deployIntake();

    }

    @Override
    public void execute() {

        if (ballSubsystem.getBottomBannerState() && !ballSubsystem.getTopBannerState()) {

            ballSubsystem.runIntake();
            ballSubsystem.runConveyor();

            Timer.delay(SuperstructureConstants.spacingDelaySeconds);

        }

    }   
    
}
