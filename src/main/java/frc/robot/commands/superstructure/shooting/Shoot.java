package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase { 

    private final ShooterSubsystem shooter;

    private double shooterVelRadPerSec;

    public Shoot(double velRadPerSec, ShooterSubsystem shoot) {

        shooter = shoot;

        shooterVelRadPerSec = velRadPerSec;

        addRequirements(shooter);

    }

    @Override
    public void initialize() {

        shooter.runKicker();

    }

    @Override
    public void execute() {

        shooter.runVelocityProfileController(shooterVelRadPerSec);

    }

    @Override
    public void end(boolean interrupted) {

        shooter.stopShooter();
        shooter.stopKicker();

    }
    
}
