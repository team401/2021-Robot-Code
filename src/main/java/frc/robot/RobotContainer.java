package frc.robot;

import javax.management.InstanceNotFoundException;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.InputDevices;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final XboxController gamepad = new XboxController(InputDevices.gamepadPort);

    private final ConveyorSubsystem conveyor = new ConveyorSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    
    public RobotContainer() {

        drive.setDefaultCommand(
            new RunCommand(
                () ->
                    drive.drive(
                        -leftJoystick.getY(GenericHID.Hand.kLeft), 
                        -leftJoystick.getX(GenericHID.Hand.kLeft), 
                        rightJoystick.getX(GenericHID.Hand.kRight), 
                        false
                    ),
                drive
            )
        );

    }

    public void configureButtonBindings() {

    }

    public Command getAutonomousCommand() {

        FollowTrajectory runTrajectory = new FollowTrajectory(drive, AutoTrajectories.autoNavSlalomTrajectory);

        drive.resetPose(runTrajectory.getInitialPose());

        return new FollowTrajectory(drive, AutoTrajectories.autoNavSlalomTrajectory);

    }
    
}