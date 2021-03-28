package frc.robot.commands.superstructure;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithVision extends CommandBase {

    private final ShooterSubsystem shooter;
    private final Limelight limelight;
    
    private final PIDController controller = new PIDController(
        0.005, 0.005, 0
        
    );
    
    public ShootWithVision(ShooterSubsystem subsystem, Limelight vision) {

        shooter = subsystem;
        limelight = vision;

        addRequirements(shooter, limelight);

    }

    @Override
    public void execute() {

        double desiredFlywheelVelRadPerSec = limelight.gettA() * SuperstructureConstants.shootingPowerConstant;
        double powerOut = controller.calculate(shooter.getFlywheelVelRadPerSec(), Units.rotationsPerMinuteToRadiansPerSecond(5000));

        shooter.runShooterPercent(powerOut);
        SmartDashboard.putNumber("rpm", Units.radiansPerSecondToRotationsPerMinute(shooter.getFlywheelVelRadPerSec()));
    
    }
    
}
