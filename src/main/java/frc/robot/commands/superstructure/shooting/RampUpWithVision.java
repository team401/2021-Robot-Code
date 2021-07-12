package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class RampUpWithVision extends CommandBase {

    /**
     * Command to ramp up the shooter based on the size of the target seen
     */

    private final ShooterSubsystem shooter;
    private final Limelight limelight;
    
    public RampUpWithVision(ShooterSubsystem subsystem, Limelight vision) {

        shooter = subsystem;
        limelight = vision;

        addRequirements(shooter, limelight);

    }

    @Override
    public void execute() {

        limelight.setLedMode(0);  // remove after testing

        //only run if the limelight has a valid lock
        if (limelight.hasValidTarget()) { 

            /*
            /**
             * tA is perentage of limelight view taken up by the target
             * We plug the tA value into a regression we generated to get a power in RMP, then use our velocity 
             * control method to ramp the shooter to the speed
             
            double ta = limelight.gettA();
            double desiredFlywheelVelRPM = 418.27 * Math.pow(ta, 5) + -4633.42 * Math.pow(ta, 4) + 19538 * Math.pow(ta, 3) + -38570.7 * Math.pow(ta, 2) + 35110.1 * ta + -6960.68;

            shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(desiredFlywheelVelRPM));
            */

            double distanceFromTargetInches = (FieldConstants.targetHeightInches - VisionConstants.limelightHeightInches) / Math.tan(VisionConstants.limelightMountAngleRadians + limelight.gettY());

            SmartDashboard.putNumber("distance in INCHES", distanceFromTargetInches);

        // otherwise, run a standard "base" shooter speed
        } else {
            
            //shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(SuperstructureConstants.baseShootingSpeed));

        }

    }

    @Override
    public void end(boolean interrupted) {

        shooter.stopKicker();

    }

}
