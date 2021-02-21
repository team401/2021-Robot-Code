package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class RobotContainer {

    private Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    private Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private DriveSubsystem drive = new DriveSubsystem();

    public RobotContainer() {

        drive.setDefaultCommand(
            new OperatorControl(
                drive,
                () -> leftJoystick.getY(GenericHID.Hand.kLeft),
                () -> leftJoystick.getX(GenericHID.Hand.kLeft),
                () -> rightJoystick.getX(GenericHID.Hand.kRight)));
    
    }

    public void configureButtonBindings() {

    }

    public Command getAutonomousCommand() {
    
        TrajectoryConfig config = 
            new TrajectoryConfig(
                AutoConstants.maxVelMetersPerSec,
                AutoConstants.maxAccelMetersPerSecondSq)
            .setKinematics(DriveConstants.kinematics);

            Trajectory trajectory = 
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)), 
                    List.of(
                        new Translation2d(1, 1), new Translation2d(2, -1)), 
                new Pose2d(3, 0, new Rotation2d(0)), 
                config);

        ProfiledPIDController rotationController = 
            new ProfiledPIDController(1, 0, 0, 
                new TrapezoidProfile.Constraints(
                    AutoConstants.maxVelMetersPerSec, 
                    AutoConstants.maxAccelMetersPerSecondSq));

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectory,
                drive::getPose,
                DriveConstants.kinematics,
                new PIDController(1, 0.0, 0.0),
                new PIDController(1, 0.0, 0.0),
                rotationController,
                drive::setModuleStates,
                drive);

        return swerveControllerCommand.andThen(() -> drive.drive(new Translation2d(0, 0), 0, false));

    }

}
