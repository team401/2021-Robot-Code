package frc.robot.commands.superstructure.indexing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;
/*establishes the indexing structure */

public class Reversing extends CommandBase {

    private final IndexingSubsystem indexer;

    private boolean isFinishedFlag = false;

    public Reversing(IndexingSubsystem subsystem) {

        indexer = subsystem;

        addRequirements(indexer);

    } //gets the state of the bottom banner sensor and schedules a new jogging command

    @Override
    public void execute() {

        boolean bottomSensorState = indexer.getBottomBannerState();

        indexer.reverseConveyor();

        if (bottomSensorState) {

            isFinishedFlag = true;
            new Jogging(indexer).schedule();

        } 

    } // Tells the robot that the indexing is finished making a new command for the robot to execute

    @Override
    public boolean isFinished() {

        return isFinishedFlag;

    }
    
}
