package frc.robot.commands.index;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.Index;

public class Jogging extends CommandBase {

    /**
     * Part of the indexing state machine
     * Spaces the ball forward a defined amount to ensure the balls don't jam together in the index
     */

    private final Index index;

    private final Timer timer = new Timer();

    public Jogging(Index subsystem) {

        index = subsystem;

        addRequirements(index);

    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

        index.runConveyor();

    }

    @Override
    public boolean isFinished() {

        return timer.get() >= SuperstructureConstants.jogDelaySeconds;
        
    }

}
