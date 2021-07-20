package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class QuickTurn extends CommandBase {

    private final DriveSubsystem drive;

    private final ProfiledPIDController controller = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(0,0));

    private Timer timer = new Timer();

    private double desiredAngle;

    public QuickTurn(double angleRad, DriveSubsystem subsystem) {

        drive = subsystem;

        desiredAngle = angleRad;

        addRequirements(drive);

    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {

        if (Math.abs(drive.getHeading().getRadians() - desiredAngle) > Units.degreesToRadians(1.5)) {

            timer.reset();

            double rotationOut = controller.calculate(drive.getHeading().getRadians(), desiredAngle);

            drive.drive(drive.getCommandedDriveValues()[0], drive.getCommandedDriveValues()[1], rotationOut, true);

        } else {

            drive.drive(drive.getCommandedDriveValues()[0], drive.getCommandedDriveValues()[1], 0, true);
            
        }

    }

    @Override
    public boolean isFinished() {

        return (Math.abs(drive.getHeading().getRadians() - desiredAngle) < Units.degreesToRadians(1.5) && timer.get() >= 0.25);

    }

    @Override
    public void end(boolean interrupted) {

        drive.drive(0, 0, 0, false);

    }
    
}