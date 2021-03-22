package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AlignWithTargetGyro extends CommandBase {

    private final DriveSubsystem drive;

    private final Rotation2d targetAngle;

    // plan is to tune this to be a more aggressive controller to get within limelight range, then profile with limelight to get to target
    private final PIDController controller = new PIDController(
        0, 0, 0
    );
    
    public AlignWithTargetGyro(DriveSubsystem subsystem, Rotation2d target) {

        drive = subsystem;

        targetAngle = target;

        controller.setTolerance(Units.degreesToRadians(20)); // Limelight has a range of 27 degrees horizontally, so this should take us into the seeable range

        addRequirements(drive);


    }

    @Override
    public void execute() {

        double rotationOut = controller.calculate(drive.getHeading().getRadians(), targetAngle.getRadians());
        
    }
    
}
