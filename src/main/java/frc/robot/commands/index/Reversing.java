package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;
/*establishes the indexing structure */

public class Reversing extends CommandBase {

    private final Index index;

    private boolean isFinishedFlag = false;

    public Reversing(Index subsystem) {

        index = subsystem;

        addRequirements(index);

    } //gets the state of the bottom banner sensor and schedules a new jogging command

    @Override
    public void execute() {

        boolean bottomSensorState = index.getBottomBannerState();

        index.reverseConveyor();

        if (bottomSensorState) {

            isFinishedFlag = true;
            new Jogging(index).schedule();

        } 

    } // Tells the robot that the indexing is finished making a new command for the robot to execute

    @Override
    public boolean isFinished() {

        return isFinishedFlag;

    }
    
}
