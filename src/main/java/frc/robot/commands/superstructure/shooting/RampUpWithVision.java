package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RampUpWithVision extends CommandBase {

    /**
     * Command to ramp up the shooter based on the size of the target seen
     */

    private final ShooterSubsystem shooter;
    private final VisionSubsystem limelight;
    
    public RampUpWithVision(ShooterSubsystem subsystem, VisionSubsystem vision) {

        shooter = subsystem;
        limelight = vision;

        addRequirements(shooter, limelight);

    }

    @Override
    public void initialize() {

        limelight.setLedMode(0);

    }

    @Override
    public void execute() {

        //only run if the limelight has a valid lock
        if (limelight.hasValidTarget()) { 
             
            double ty = limelight.gettY();
            double desiredFlywheelVelRPM = 12637 * Math.pow(ty, 2) + 3201.4 * ty + 4057.3;

            shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(desiredFlywheelVelRPM));

        // otherwise, run a standard "base" shooter speed
        } else {
            
            shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(SuperstructureConstants.baseShootingSpeed));

        }

    }

    @Override
    public void end(boolean interrupted) {

        shooter.stopKicker();
        shooter.runShooterPercent(0.0);
        
        limelight.setLedMode(1);

    }

}
