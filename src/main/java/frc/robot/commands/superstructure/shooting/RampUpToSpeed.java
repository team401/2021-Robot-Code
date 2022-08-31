package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RampUpToSpeed extends CommandBase {
    
    private final ShooterSubsystem shooter;

    private final IndexingSubsystem indexer; //not used by this command, is to be passed into Shoot 

    private double desiredSpeed;

    public RampUpToSpeed(double desiredSpeedRadPerSec, ShooterSubsystem shoot, IndexingSubsystem indexer) {

        shooter = shoot;

        this.indexer = indexer;

        desiredSpeed = desiredSpeedRadPerSec;

        addRequirements(shooter);

    }

    @Override
    public void execute() {

        shooter.runVelocityProfileController(desiredSpeed);

    }

    @Override
    public void end(boolean inturrupted) {
        if (inturrupted) {
            shooter.stopShooter();
        }
        else {
            new Shoot(shooter, indexer).schedule();
        }
    }

    @Override
    public boolean isFinished() {

        return shooter.getFlywheelVelRadPerSec() >= desiredSpeed;

    }
    
}
