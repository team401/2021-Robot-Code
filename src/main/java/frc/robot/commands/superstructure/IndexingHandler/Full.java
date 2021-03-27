package frc.robot.commands.superstructure.IndexingHandler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

public class Full extends CommandBase {

    private final IndexingSubsystem indexer;

    private boolean isFinshedFlag = false;

    public Full(IndexingSubsystem subsystem) {

        indexer = subsystem;

        addRequirements(indexer);

    }

    @Override
    public void execute() {

        boolean bottomSensorState = indexer.getBottomBannerState();
        boolean topSensorState = indexer.getTopBannerState();

        indexer.stopConveyor();

        if (!bottomSensorState || !topSensorState) {

            new Waiting(indexer).schedule();
            isFinshedFlag = true;

        }

    }

    @Override
    public boolean isFinished() {

        return isFinshedFlag;

    }
    
}
