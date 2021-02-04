package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.OperatorControl;

public class RobotContainer {

    private Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    private Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final OperatorControl operatorControl = new OperatorControl();

    public RobotContainer() { }

    public Joystick getLeftJoystick() {

        return leftJoystick;

    }

    public Joystick getRightJoystick() {

        return rightJoystick;

    }
}
