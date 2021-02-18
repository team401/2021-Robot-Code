package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.drivetrain.FollowTrajectoryCommand;
import frc.robot.commands.drivetrain.OperatorControlCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

    private Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    private Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private DriveSubsystem drive = new DriveSubsystem();

    public RobotContainer() {

        drive.setDefaultCommand(
            new OperatorControlCommand(
                drive,
                () -> leftJoystick.getY(GenericHID.Hand.kLeft),
                () -> leftJoystick.getX(GenericHID.Hand.kLeft),
                () -> rightJoystick.getX(GenericHID.Hand.kRight)));
    
    }

    public void configureButtonBindings() {

    }

    public Command getAutonomousCommand() {

        return new FollowTrajectoryCommand().runCommand();


    }

}
