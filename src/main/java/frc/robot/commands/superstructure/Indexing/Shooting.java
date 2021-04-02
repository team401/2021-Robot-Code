package frc.robot.commands.superstructure.indexing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;

public class Shooting extends CommandBase {

    private final IndexingSubsystem indexer;

    public Shooting(IndexingSubsystem index) {

        indexer = index;

        addRequirements(indexer);

    }

    @Override
    public void execute() {

        indexer.runConveyor();

    }
    
}

