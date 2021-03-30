package frc.robot.commands.superstructure.indexing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.IndexingSubsystem;

public class Jogging extends CommandBase {

    private final IndexingSubsystem indexer;

    private final Timer timer = new Timer();

    public Jogging(IndexingSubsystem subsystem) {

        indexer = subsystem;

        addRequirements(indexer);

    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

        indexer.runConveyor();

    }

    @Override
    public void end(boolean interrupted) {

        new Waiting(indexer).schedule();

    }

    @Override
    public boolean isFinished() {

        return timer.get() >= SuperstructureConstants.jogDelaySeconds;
        
    }

}
