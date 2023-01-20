package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

public class Full extends CommandBase {

    /**
     * Part of the indexing state machine
     * Command that runs when both sensors read true (meaning the index is full)
     * Exits when either of the sensors become false, meaning the index is no longer full
     */

    private final Index index;

    private boolean isFinshedFlag = false;

    public Full(Index subsystem) {

        index = subsystem;

        addRequirements(index);

    }

    @Override
    public void execute() {

        boolean bottomSensorState = index.getBottomBannerState();
        boolean topSensorState = index.getTopBannerState();

        index.stopConveyor();

        if (!bottomSensorState || !topSensorState) 
            isFinshedFlag = true;

    }

    @Override
    public boolean isFinished() {

        return isFinshedFlag;

    }
    
}
