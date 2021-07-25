package frc.robot.commands.superstructure.indexing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

/* Establishes the waiting structure */

public class Waiting extends CommandBase {

    private final IndexingSubsystem indexer;

    private boolean isFinishedFlag = false;

    public Waiting(IndexingSubsystem subsystem) {

        indexer = subsystem;

        addRequirements(indexer);

    }
    
/* If one of the banner sensors state is set to active then start running to the point where it is clear,
 if BOTH are active then stop running the intake */

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
