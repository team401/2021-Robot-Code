package frc.robot.commands.superstructure.Indexing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

public class Waiting extends CommandBase {

    private final IndexingSubsystem indexer;

    private boolean isFinishedFlag = false;

    public Waiting(IndexingSubsystem subsystem) {

        indexer = subsystem;

        addRequirements(indexer);

    }
    
    @Override
    public void execute() {

        boolean bottomSensorState = indexer.getBottomBannerState();
        boolean topSensorState = indexer.getTopBannerState();

        indexer.stopConveyor();    

        if (bottomSensorState && !topSensorState) {

            isFinishedFlag = true;
            new Feeding(indexer).schedule();

        } else if (!bottomSensorState && topSensorState) {

            isFinishedFlag = true;
            new Reversing(indexer).schedule();

        } else if (bottomSensorState && topSensorState) {

            isFinishedFlag = true;
            new Full(indexer).schedule();

        }

    }

    @Override
    public boolean isFinished() {

        return isFinishedFlag;

    }

}
