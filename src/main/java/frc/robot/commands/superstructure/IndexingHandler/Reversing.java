package frc.robot.commands.superstructure.IndexingHandler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

public class Reversing extends CommandBase {

    private final IndexingSubsystem indexer;

    private boolean isFinishedFlag =  false;

    public Reversing(IndexingSubsystem subsystem) {

        indexer = subsystem;

        addRequirements(indexer);

    }

    @Override
    public void execute() {

        boolean bottomSensorState = indexer.getBottomBannerState();

        indexer.reverseConveyor();

        if (bottomSensorState) {

            new Waiting(indexer).schedule();
            isFinishedFlag = true;

        } 

    }

    @Override
    public boolean isFinished() {

        return isFinishedFlag;

    }
    
}
