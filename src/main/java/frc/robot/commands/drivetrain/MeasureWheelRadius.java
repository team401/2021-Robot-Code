package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class MeasureWheelRadius extends CommandBase {

    /**
     * Command to measure the wheel radius of the drive
     * For swerve, we assume each module wheel radius to be the same and use the average distance driven as the robot distance
     */

    private final DriveSubsystem drive;

    private double distance;

    public MeasureWheelRadius(double distanceMeters, DriveSubsystem subsystem) {

        drive = subsystem;

        distance = distanceMeters;

    }

    @Override
    public void initialize() {

        drive.resetDriveDistances();

    }

    @Override
    public void end(boolean interrupted) {

        double distanceRadians = drive.getAverageDriveDistanceRadians();

        SmartDashboard.putNumber("wheel radius meters", distance / distanceRadians);

    }

}
