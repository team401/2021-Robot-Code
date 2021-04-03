package frc.robot.commands.superstructure.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
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

        SmartDashboard.putNumber("ta", limelight.gettA());

        if (limelight.hasValidTarget()) { 

            double ta = limelight.gettA();
            double desiredFlywheelVelRPM = 418.27 * Math.pow(ta, 5) + -4633.42 * Math.pow(ta, 4) + 19538 * Math.pow(ta, 3) + -38570.7 * Math.pow(ta, 2) + 35110.1 * ta + -6960.68;

            shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(desiredFlywheelVelRPM));

        } else {
            
            shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(SuperstructureConstants.baseShootingSpeed));

        }

    }

    @Override
    public void end(boolean interrupted) {

        shooter.stopKicker();

    }

}
