package frc.robot.commands.superstructure.IndexingHandler;

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

            new Feeding(indexer).schedule();
            isFinishedFlag = true;

        } else if (!bottomSensorState && topSensorState) {

            new Reversing(indexer).schedule();
            isFinishedFlag = true;

        }

    }

    @Override
    public boolean isFinished() {

        return isFinishedFlag;

    }

}
