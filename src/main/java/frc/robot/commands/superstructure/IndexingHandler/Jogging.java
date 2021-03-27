package frc.robot.commands.superstructure.IndexingHandler;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.IndexingSubsystem;

public class Jogging extends CommandBase {

    private final IndexingSubsystem indexer;

    private final Timer timer = new Timer();

    public Jogging(IndexingSubsystem subsystem) {

        indexer = subsystem;

        addRequirements(indexer);

    }

    @Override
    public void initialize() {
        
        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {

    while (timer.get() < SuperstructureConstants.jogForwardTime) 
    
        indexer.runConveyor();
    
    }

    @Override
    public void end(boolean interrupted) {

        new Waiting(indexer).schedule();
        SmartDashboard.putString("done", "going to waiting");

    }

    @Override
    public boolean isFinished() {

        return timer.get() < SuperstructureConstants.jogForwardTime;
        
    }

}
    

