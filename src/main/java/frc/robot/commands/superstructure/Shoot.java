package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {

    private final IndexingSubsystem indexer;
    private final ShooterSubsystem shooter;

    public Shoot(IndexingSubsystem index, ShooterSubsystem shoot) {

        indexer = index;
        shooter = shoot;

        addRequirements(indexer, shooter);

    }

    @Override
    public void execute() {

        shooter.runKicker();
        indexer.runConveyor();

    }

    @Override
    public void end(boolean interrupted) {

        shooter.stopKicker();
        indexer.stopConveyor();

    }
    
}
