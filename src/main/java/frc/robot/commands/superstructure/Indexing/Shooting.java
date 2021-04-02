package frc.robot.commands.superstructure.indexing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shooting extends CommandBase {

    private final IndexingSubsystem indexer;
    private final ShooterSubsystem shooter;

    public Shooting(IndexingSubsystem index, ShooterSubsystem shoot) {

        indexer = index;
        shooter = shoot;

        addRequirements(indexer, shooter);

    }

    @Override
    public void execute() {

        if (shooter.atGoal()) indexer.runConveyor();
        else indexer.stopConveyor();

    }
    
}

