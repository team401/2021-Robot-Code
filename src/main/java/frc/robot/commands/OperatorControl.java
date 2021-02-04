package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

import org.frcteam2910.common.robot.Utilities;

public class OperatorControl extends Command {

    public OperatorControl() {

        requires(DriveSubsystem.getInstance());

    }

    @Override
    protected void execute() {

        double forward = Utilities.deadband(-Robot.getContainer().getLeftJoystick().getRawAxis(1));
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = Utilities.deadband(-Robot.getContainer().getLeftJoystick().getRawAxis(0));
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation = Utilities.deadband(-Robot.getContainer().getRightJoystick().getRawAxis(0));
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

        DriveSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation);

    }

    @Override
    protected boolean isFinished() {

        return false;

    }
}
