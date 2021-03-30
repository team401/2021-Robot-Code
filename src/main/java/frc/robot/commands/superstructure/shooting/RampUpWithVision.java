package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class RampUpWithVision extends CommandBase {

    private final ShooterSubsystem shooter;
    private final Limelight limelight;
    
    private final ProfiledPIDController controller = new ProfiledPIDController(
        0.005, 0.005, 0,
        new TrapezoidProfile.Constraints(
            Units.rotationsPerMinuteToRadiansPerSecond(6380) * SuperstructureConstants.flywheelGearRatio,
            Units.rotationsPerMinuteToRadiansPerSecond(2000)
        )
    );
    
    public RampUpWithVision(ShooterSubsystem subsystem, Limelight vision) {

        shooter = subsystem;
        limelight = vision;

        addRequirements(shooter, limelight);

    }

    @Override
    public void execute() {

        if (limelight.hasValidTarget()) {

            double ta = limelight.gettA();

            double desiredFlywheelVelRadPerSec = (1803.29 * Math.pow(ta, 2)) - (4540.2 * ta) + 8289.72;

            double powerOut = controller.calculate(
                shooter.getFlywheelVelRadPerSec(), 
                Units.rotationsPerMinuteToRadiansPerSecond(desiredFlywheelVelRadPerSec)
            );
            
            shooter.runShooterPercent(powerOut);

        } else {

            double powerOut = controller.calculate(
                shooter.getFlywheelVelRadPerSec(),
                Units.rotationsPerMinuteToRadiansPerSecond(SuperstructureConstants.baseShootingSpeed)
            );

            shooter.runShooterPercent(powerOut);

        }
        
    }

    public boolean getAtGoal() {

        return controller.atGoal();

    }

}