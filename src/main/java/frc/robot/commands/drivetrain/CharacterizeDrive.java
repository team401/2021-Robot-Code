package frc.robot.commands.drivetrain;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class CharacterizeDrive extends CommandBase {

    /**
     * Runs along with the FRC Characterziation Tool to generate feedforward gains for the drivetrain
     */

    private final DriveSubsystem drive;

    private final NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    private final NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    
    private final double[] telemetryArray = new double[6];

    private double priorAutoSpeed = 0;

    public CharacterizeDrive(DriveSubsystem subsystem) {

        drive = subsystem;

        addRequirements(drive);

    }

    @Override
    public void initialize() {

        drive.resetDriveDistances();

        priorAutoSpeed = 0;

        NetworkTableInstance.getDefault().setUpdateRate(0.01);

    }

    @Override
    public void execute() {

        double now = Timer.getFPGATimestamp();

        double averageDistanceRadians = drive.getAverageDriveDistanceRadians();
        double averageVelocityRadPerSec = drive.getAverageDriveVelocityRadiansPerSecond();

        double batteryVoltage = RobotController.getBatteryVoltage();
        double motorVoltage = batteryVoltage * Math.abs(priorAutoSpeed);

        double autoSpeedCurrent = autoSpeedEntry.getDouble(0);
        priorAutoSpeed = autoSpeedCurrent;

        drive.drive(autoSpeedCurrent, 0, 0, false);

        telemetryArray[0] = now;
        telemetryArray[1] = batteryVoltage;
        telemetryArray[2] = autoSpeedCurrent;
        telemetryArray[3] = motorVoltage;
        telemetryArray[4] = averageDistanceRadians;
        telemetryArray[5] = averageVelocityRadPerSec;

        telemetryEntry.setDoubleArray(telemetryArray);

    }

}
