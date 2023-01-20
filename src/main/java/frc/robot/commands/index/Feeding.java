package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

public class Feeding extends CommandBase {

    /**
     * Part of the indexing state machine
     * Command that runs the index foward when a ball is first seen, then exits when either the index is full,
     * a ball reaches the top of the index, or the ball passes the bottom sensor (meaning the ball has been successfully indexed)
     */

    private final Index index;

    private boolean isFinishedFlag = false;

    public Feeding(Index subsystem) {

        index = subsystem;

        addRequirements(index);

    }

    @Override
    public void execute() {

        boolean bottomSensorState = index.getBottomBannerState();
        boolean topSensorState = index.getTopBannerState();

        if (bottomSensorState && topSensorState) {

            isFinishedFlag = true;
            new Full(index).schedule();

        } else if (topSensorState) {
            
            isFinishedFlag = true;
            new Reversing(index).schedule();

        } else if (!bottomSensorState) {

            isFinishedFlag = true;
            new Jogging(index).schedule();

        } else {

            index.runConveyor();

        }

    }

    @Override
    public boolean isFinished() {

        return isFinishedFlag;

    }
    
}
