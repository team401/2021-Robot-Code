package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.drivetrain.AlignWithGyro;
import frc.robot.commands.drivetrain.AlignWithTargetVision;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.commands.superstructure.Intake;
import frc.robot.commands.superstructure.Shoot;
import frc.robot.commands.superstructure.indexing.Waiting;
import frc.robot.commands.superstructure.shooting.RampUpWithVision;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final XboxController gamepad = new XboxController(InputDevices.gamepadPort);

    private DriveSubsystem drive = new DriveSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private IndexingSubsystem indexer = new IndexingSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private Limelight limelight = new Limelight();
    
    public RobotContainer() {

        drive.setDefaultCommand(
            new OperatorControl(
                drive, 
                () -> leftJoystick.getX(GenericHID.Hand.kLeft), 
                () -> leftJoystick.getY(GenericHID.Hand.kLeft), 
                () -> rightJoystick.getX(GenericHID.Hand.kRight),
                true
            )
        );

        indexer.setDefaultCommand(new Waiting(indexer));

        configureButtonBindings();

    }

    public void configureButtonBindings() {

        // intake
        new JoystickButton(gamepad, Button.kB.value)
            .whenPressed(new Intake(intake));

        // ramp up shooter using vision
        new JoystickButton(gamepad, Button.kBumperRight.value)
            .whenPressed(new RampUpWithVision(shooter, limelight));

        // shoot
        new JoystickButton(gamepad, Button.kY.value)
            .whileHeld(new Shoot(indexer, shooter));

        // manual reverse
        new JoystickButton(gamepad, Button.kBack.value) 
            .whileHeld(
                new ParallelCommandGroup(
                    new InstantCommand(shooter::reverseKicker, shooter),
                    new InstantCommand(indexer::reverseConveyor, indexer),
                    new InstantCommand(intake::reverseIntakeMotor, intake)
                )
            )
            .whenReleased(
                new ParallelCommandGroup(
                    new InstantCommand(shooter::stopKicker, shooter),
                    new InstantCommand(indexer::stopConveyor, indexer),
                    new InstantCommand(intake::stopIntakeMotor, intake)
                )
            );

            // shoot close
            new POVButton(gamepad, 0)
                .whileHeld(
                    new InstantCommand(
                        () -> shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(4500))
                    )
                )
                .whenReleased(new InstantCommand(shooter::stopShooter));

            // shoot mid-close
            new POVButton(gamepad, 90)
                .whileHeld(
                    new InstantCommand(
                        () -> shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(3800))
                    )
                )
                .whenReleased(new InstantCommand(shooter::stopShooter));

            // shoot mid-far
            new POVButton(gamepad, 180)
                .whileHeld(
                    new InstantCommand(
                        () -> shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(6000))
                    )
                    .alongWith(new InstantCommand(shooter::extendHood))
                )
                .whenReleased(
                    new InstantCommand(shooter::stopShooter)
                );

            // shoot far
            new POVButton(gamepad, 270)
                .whileHeld(
                    new InstantCommand(
                        () -> shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(6300))
                    )
                    .alongWith(new InstantCommand(shooter::extendHood, shooter))
                )
                .whenReleased(
                    new InstantCommand(shooter::stopShooter)
                );

        // align with vision
        new JoystickButton(leftJoystick, Joystick.ButtonType.kTop.value)
            .whileHeld(new AlignWithTargetVision(drive, limelight));

        // align with 0 degrees
        new JoystickButton(rightJoystick, Joystick.ButtonType.kTrigger.value)
            .whileHeld(new AlignWithGyro(drive, Math.PI)
        );

        // align with 180 degrees
        new JoystickButton(leftJoystick, Joystick.ButtonType.kTrigger.value) 
            .whileHeld(new AlignWithGyro(drive, 0)
        );

        // reset imu 
        new JoystickButton(rightJoystick, Joystick.ButtonType.kTop.value)
            .whenPressed(new InstantCommand(drive::resetImu));

        new JoystickButton(gamepad, Button.kX.value)
            .whenPressed(new InstantCommand(shooter::toggleHood));

    }

    public Command getAutonomousCommand() {

        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.maxVelMetersPerSec, 
            AutoConstants.maxAccelMetersPerSecondSq
        )
        .setKinematics(DriveConstants.kinematics);

        config.setStartVelocity(0.0);
        config.setEndVelocity(0.0);

        FollowTrajectory runTrajectory = new FollowTrajectory(drive, AutoTrajectories.autoNavBarrelTrajectory);

        drive.resetPose(new Pose2d(runTrajectory.getInitialPose().getX(), runTrajectory.getInitialPose().getY(), Rotation2d.fromDegrees(90)));

        return new FollowTrajectory(drive, AutoTrajectories.autoNavBarrelTrajectory);

    }

}
