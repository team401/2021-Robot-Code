package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RampUpToSpeed extends CommandBase {
    
    private final ShooterSubsystem shooter;

    private double desiredSpeed;

    public RampUpToSpeed(double desiredSpeedRadPerSec, ShooterSubsystem shoot) {

        shooter = shoot;

        desiredSpeed = desiredSpeedRadPerSec;

        addRequirements(shooter);

    }

    @Override
    public void execute() {

        shooter.runVelocityProfileController(desiredSpeed);

    }

    @Override
    public boolean isFinished() {

        return shooter.atGoal();

    }
    
}
