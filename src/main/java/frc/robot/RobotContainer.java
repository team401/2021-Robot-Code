package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.OperatorControl;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

    private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final DriveSubsystem drive = new DriveSubsystem();

    private final OperatorControl operatorControl = 
        new OperatorControl(
            drive, 
            () -> leftJoystick.getX(GenericHID.Hand.kLeft), 
            () -> leftJoystick.getY(GenericHID.Hand.kLeft), 
            () -> rightJoystick.getX(GenericHID.Hand.kRight));

    public RobotContainer() {

        drive.setDefaultCommand(operatorControl);

        configureButtonBindings();

    }

    private void configureButtonBindings() {

    }

    public Command getAutonomousCommand() {

        return null;

    }

}
