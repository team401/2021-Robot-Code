package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class QuickTurn extends CommandBase {

    /**
     * A command to align the robot automatically to a certain heading, used for a "quick turn" for shooting
     */

    private final DriveSubsystem drive;

    private final double desiredAngle;
    private double rotationOut = 0;

    private final PIDController controller = new PIDController(
        5, 0, 0
    );

    private final Timer timer = new Timer();

    public QuickTurn(double desiredAngleRad, DriveSubsystem subsystem) {

        drive = subsystem;

        desiredAngle = desiredAngleRad;

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

            rotationOut = controller.calculate(drive.getHeading().getRadians(), desiredAngle);


        } else {

            rotationOut = 0;
            
        }

        drive.drive(
            drive.getCommandedDriveValues()[0], 
            drive.getCommandedDriveValues()[1], 
            rotationOut, 
            drive.getIsFieldRelative()
        );


    }
    
    @Override
    public boolean isFinished() {

        return (Math.abs(drive.getHeading().getRadians() - desiredAngle) < Units.degreesToRadians(1.5) && timer.get() >= 0.25);

    }

}