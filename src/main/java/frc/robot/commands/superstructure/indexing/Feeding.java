package frc.robot.commands.superstructure.indexing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

public class Feeding extends CommandBase {

    /**
     * Part of the indexing state machine
     * Command that runs the indexer foward when a ball is first seen, then exits when either the indexer is full,
     * a ball reaches the top of the indexer, or the ball passes the bottom sensor (meaning the ball has been successfully indexed)
     */

    private final IndexingSubsystem indexer;

    private boolean isFinishedFlag = false;

    public Feeding(IndexingSubsystem subsystem) {

        indexer = subsystem;

        addRequirements(indexer);

    }

    @Override
    public void execute() {

        boolean bottomSensorState = indexer.getBottomBannerState();
        boolean topSensorState = indexer.getTopBannerState();

        if (bottomSensorState && topSensorState) {

            isFinishedFlag = true;
            new Full(indexer).schedule();

        } else if (topSensorState) {
            
            isFinishedFlag = true;
            new Reversing(indexer).schedule();

        } else if (!bottomSensorState) {

            isFinishedFlag = true;
            new Jogging(indexer).schedule();

        } else {

            indexer.runConveyor();

        }

    }

    @Override
    public boolean isFinished() {

        return isFinishedFlag;

    }
    
}
