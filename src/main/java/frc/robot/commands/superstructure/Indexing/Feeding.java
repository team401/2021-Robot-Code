package frc.robot.commands.superstructure.indexing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

public class Feeding extends CommandBase {

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
