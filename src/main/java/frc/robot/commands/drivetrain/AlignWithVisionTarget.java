package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignWithVisionTarget extends CommandBase {

    private final DriveSubsystem drive;
    private final VisionSubsystem limelight;

    private final PIDController controller = new PIDController(
        5, 0, 0
    );

    private Timer timer = new Timer();

    private double rotationOut = 0;
    
    public AlignWithVisionTarget(DriveSubsystem subsystem, VisionSubsystem vision) {

        drive = subsystem;
        limelight = vision;

    }

    @Override
    public void initialize() {

        limelight.setLedMode(0);

        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {

        if (limelight.hasValidTarget()) {

            if (Math.abs(limelight.gettX()) > Units.degreesToRadians(1.5)) {

                rotationOut = controller.calculate(limelight.gettX(), 0);

            } else {

                rotationOut = 0;

            }

            drive.drive(
                drive.getCommandedDriveValues()[0], 
                drive.getCommandedDriveValues()[1], 
                rotationOut, 
                drive.getIsFieldRelative()
            );

        } else {

            timer.reset();

            drive.drive(
                drive.getCommandedDriveValues()[0], 
                drive.getCommandedDriveValues()[1], 
                drive.getCommandedDriveValues()[2], 
                drive.getIsFieldRelative()
            );

        }

    }

    @Override
    public boolean isFinished() {

        return limelight.hasValidTarget() && (Math.abs(limelight.gettX()) < Units.degreesToRadians(1.5) && timer.get() >= 0.5);

    }

    @Override
    public void end(boolean interrupted) {

        limelight.setLedMode(1);

    }

}