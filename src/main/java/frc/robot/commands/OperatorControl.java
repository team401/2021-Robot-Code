package frc.robot.commands;

import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class OperatorControl extends CommandBase {

    private final DriveSubsystem drive;

    private double forward;
    private double strafe;
    private double rotation;
    
    public OperatorControl(DriveSubsystem subsystem) {

        drive = subsystem;
        addRequirements(drive);

    }

    @Override
    public void execute() {

        forward = Utilities.deadband(Robot.getContainer().getLeftJoystick().getX(Hand.kLeft));
        forward = Math.copySign(Math.pow(forward, 2.0), forward);


        strafe = Utilities.deadband(Robot.getContainer().getLeftJoystick().getY(Hand.kLeft));
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        rotation = Utilities.deadband(Robot.getContainer().getRightJoystick().getX(Hand.kRight));
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

        drive.drive(new Translation2d(forward, strafe), rotation, false);

    }

}
