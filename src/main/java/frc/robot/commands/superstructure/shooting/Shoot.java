package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {

    private final ShooterSubsystem shooter;
    private final IndexingSubsystem indexer;

    private final Timer timer = new Timer();

    public Shoot(ShooterSubsystem shoot, IndexingSubsystem index) {

        shooter = shoot;
        indexer = index;

        addRequirements(shooter, indexer);

    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {

        if (shooter.atGoal()) {

            shooter.runKicker();
            indexer.runConveyor();

        } else { 

            shooter.stopKicker();
            indexer.stopConveyor();

            timer.reset();

        }

    }

    @Override
    public boolean isFinished() {

        return timer.get() > 2;

    }

    @Override
    public void end(boolean interrupted) {

        shooter.stopShooter();
        shooter.stopKicker();
        indexer.stopConveyor();

    }
    
}
