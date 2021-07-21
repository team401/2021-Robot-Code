package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RampUpWithVision extends CommandBase {

    /**
     * Command to ramp up the shooter based on the seen vision angle offset
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

        SmartDashboard.putNumber("tY Value", limelight.gettY());

        //only run if the limelight has a valid lock
        /*if (limelight.hasValidTarget()) { 
             
            double ty = limelight.gettY();
            double desiredFlywheelVelRPM = 7378.13 * Math.pow(ty, 2) + -2200.66 * ty + 4079.94;

            shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(desiredFlywheelVelRPM));

        // otherwise, run a standard "base" shooter speed
        } else {
            
            shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(SuperstructureConstants.baseShootingSpeed));

        }*/

        shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(SmartDashboard.getNumber("desired RPM", 0)));

    }

    /*@Override
    public boolean isFinished() {

        return shooter.atGoal();

    }

    @Override
    public void end(boolean interrupted) {
        
        limelight.setLedMode(1);

    }*/

}
