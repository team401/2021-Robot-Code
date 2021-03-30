package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class RampUpWithVision extends CommandBase {

    private final ShooterSubsystem shooter;
    private final Limelight limelight;
    
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

            shooter.runVelocityProfileController(desiredFlywheelVelRadPerSec);

        } else {
            
            shooter.runVelocityProfileController(SuperstructureConstants.baseShootingSpeed);

        }
        
    }

}
