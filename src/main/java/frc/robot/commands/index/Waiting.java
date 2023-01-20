package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

public class Waiting extends CommandBase {

    private final Index index;

    private boolean isFinishedFlag = false;

    public Waiting(Index subsystem) {

        index = subsystem;

        addRequirements(index);

    }
    
/* If one of the banner sensors state is set to active then start running to the point where it is clear,
 if BOTH are active then stop running the intake */

    @Override
    public void execute() {

        boolean bottomSensorState = index.getBottomBannerState();
        boolean topSensorState = index.getTopBannerState();

        

        index.stopConveyor();    

        if (bottomSensorState && !topSensorState) {

            isFinishedFlag = true;
            new Feeding(index).schedule();

        } else if (!bottomSensorState && topSensorState) {

            isFinishedFlag = true;
            new Reversing(index).schedule();

        } else if (bottomSensorState && topSensorState) {

            isFinishedFlag = true;
            new Full(index).schedule();

        }

    }

    @Override
    public boolean isFinished() {

        return isFinishedFlag;

    }

}
