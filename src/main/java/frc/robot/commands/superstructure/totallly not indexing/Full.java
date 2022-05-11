package frc.robot.commands.superstructure.indexing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

public class Full extends CommandBase {

    /**
     * Part of the indexing state machine
     * Command that runs when both sensors read true (meaning the indexer is full)
     * Exits when either of the sensors become false, meaning the indexer is no longer full
     */

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

        if (!bottomSensorState || !topSensorState) 
            isFinshedFlag = true;

    }

    @Override
    public boolean isFinished() {

        return isFinshedFlag;

    }
    
}
