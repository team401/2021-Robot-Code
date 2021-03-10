package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.InputDevices;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

    private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final XboxController gamepad = new XboxController(InputDevices.gamepadPort);

    private DriveSubsystem drive = new DriveSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ConveyorSubsystem conveyor = new ConveyorSubsystem();
    
    public RobotContainer() {

        drive.setDefaultCommand(
            new RunCommand(() ->
                drive.drive(
                    -leftJoystick.getY(GenericHID.Hand.kLeft), 
                    -leftJoystick.getX(GenericHID.Hand.kLeft), 
                    rightJoystick.getX(GenericHID.Hand.kRight),
                    true
                ),
                drive
            )

        );

    }

    public void configureButtonBindings() {

        new JoystickButton(gamepad, Button.kB.value)
            .whenPressed(intake::extendIntake)
            .whenReleased(intake::retractIntake)
            .whileHeld(intake::runIntakeMotor)
            .whileHeld(conveyor::runConveyor);

    }

    public Command getAutonomousCommand() {

        FollowTrajectory runTrajectory = new FollowTrajectory(drive, AutoTrajectories.trajectory);

        //drive.resetPose(runTrajectory.getInitialPose());
        SmartDashboard.putString("hell yeah he's cool", "I'm mary poppins yall");

        return runTrajectory;

    }
}
