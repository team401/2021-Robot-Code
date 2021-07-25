package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTime extends CommandBase {

    private final DriveSubsystem drive;

    private double driveSeconds;
    private double drivePower;

    private Timer timer = new Timer();

    public DriveTime(double seconds, double power, DriveSubsystem subsystem) {

        driveSeconds = seconds;
        drivePower = power;

        drive = subsystem;

    }

    @Override
    public void initialize() {

        timer.start();
        timer.reset();

    }

    @Override
    public void execute() {

        drive.drive(drivePower, 0, 0, false);

    }

    @Override
    public boolean isFinished() {

        return timer.get() >= driveSeconds;

    }
    
}
