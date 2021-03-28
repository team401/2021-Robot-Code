package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {

    private final ShooterSubsystem shooter;
    
    private final double desiredFlywheelSpeed;

    private final ProfiledPIDController controller = new ProfiledPIDController(
        1, 0, 0, 
        new TrapezoidProfile.Constraints(
            Units.rotationsPerMinuteToRadiansPerSecond(6380) / SuperstructureConstants.flywheelGearRatio,
            Units.rotationsPerMinuteToRadiansPerSecond(5)
        )
    );

    public Shoot(double speed, ShooterSubsystem subsystem) {

        desiredFlywheelSpeed = speed;
        shooter = subsystem;

        addRequirements(shooter);

    }

    @Override
    public void execute() {

        double powerOut = controller.calculate(shooter.getFlywheelVelRadPerSec(), desiredFlywheelSpeed);

        shooter.runShooterPercent(powerOut);

    }
    
}
