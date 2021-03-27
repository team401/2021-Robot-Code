package frc.robot.commands.superstructure.IndexingHandler;

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

        indexer.runConveyor();

        if (bottomSensorState && topSensorState) {

            new Full(indexer).schedule();
            isFinishedFlag = true;

        } else if (topSensorState) {

            new Reversing(indexer).schedule();
            isFinishedFlag = true;

        } else if (!bottomSensorState) {

            new Jogging(indexer).schedule();
            isFinishedFlag = true;

        }

    }

    @Override
    public boolean isFinished() {

        return isFinishedFlag;

    }
    
}
