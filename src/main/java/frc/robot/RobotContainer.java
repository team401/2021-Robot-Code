package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.List;

public class RobotContainer {

    private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final XboxController gamepad = new XboxController(InputDevices.gamepadPort);

    private DriveSubsystem drive = new DriveSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();

    public RobotContainer() {

        drive.setDefaultCommand(
            new OperatorControl(
                drive,
                () -> leftJoystick.getY(GenericHID.Hand.kLeft),
                () -> leftJoystick.getX(GenericHID.Hand.kLeft),
                () -> rightJoystick.getX(GenericHID.Hand.kRight)));
    
    }

    public void configureButtonBindings() {

        new JoystickButton(gamepad, Button.kB.value)
        .whenHeld(new InstantCommand(intake::extendIntake))
        .whenReleased(new InstantCommand(intake::retractIntake));

    }

    public Command getAutonomousCommand() {
    
        TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.maxVelMetersPerSec,
                    AutoConstants.maxAccelMetersPerSecondSq)
                .setKinematics(DriveConstants.kinematics);

        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var rotationController =
            new ProfiledPIDController(
                2, 0.0, 0,
                new TrapezoidProfile.Constraints(
                    AutoConstants.maxAngularSpeedRadPerSec,
                    AutoConstants.maxAngularAccelRadPerSecSq));
        //rotationController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                drive::getPose,
                DriveConstants.kinematics,
                new PIDController(1, 0, 0.25),
                new PIDController(1, 0, 0.25),
                rotationController,
                drive::setModuleStates,
                drive);

        drive.resetPose(exampleTrajectory.getInitialPose());

        return swerveControllerCommand;

    }

}
