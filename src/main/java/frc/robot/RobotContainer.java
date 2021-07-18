package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.InputDevices;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.drivetrain.QuickTurn;
import frc.robot.commands.climbing.ActuateClimbers;
import frc.robot.commands.drivetrain.AlignWithTargetVision;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.commands.superstructure.indexing.Waiting;
import frc.robot.commands.superstructure.shooting.RampUpWithVision;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

    private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final XboxController gamepad = new XboxController(InputDevices.gamepadPort);

    private DriveSubsystem drive = new DriveSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private IndexingSubsystem indexer = new IndexingSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private Limelight limelight = new Limelight();
    private ClimbingSubsystem climber = new ClimbingSubsystem();
    
    public RobotContainer() {
        drive.setDefaultCommand(
            new OperatorControl(
                drive, 
                () -> leftJoystick.getY(GenericHID.Hand.kLeft), 
                () -> leftJoystick.getX(GenericHID.Hand.kLeft), 
                () -> rightJoystick.getX(GenericHID.Hand.kRight),
                true
            )
        );

        climber.setDefaultCommand(
            new ActuateClimbers(
                climber,
                () -> gamepad.getRawAxis(5)
            )
        );

        indexer.setDefaultCommand(new Waiting(indexer));

        configureButtonBindings();

    }

    public void configureButtonBindings() {

        // intake
        new JoystickButton(gamepad, Button.kB.value)
            .whenPressed(new InstantCommand(intake::runIntakeMotor))
            .whenReleased(new InstantCommand(intake::stopIntakeMotor));

        // shoot
        new JoystickButton(gamepad, Button.kY.value)
            .whileHeld(new InstantCommand(shooter::runKicker)
            .alongWith(new InstantCommand(indexer::runConveyor, indexer)))
            .whenReleased(new InstantCommand(shooter::stopKicker)
            .alongWith(new InstantCommand(indexer::stopConveyor, indexer)));

        // manual reverse
        new JoystickButton(gamepad, Button.kBack.value) 
            .whileHeld(
                new ParallelCommandGroup(
                    new InstantCommand(shooter::reverseKicker),
                    new InstantCommand(indexer::reverseConveyor, indexer),
                    new InstantCommand(intake::reverseIntakeMotor)
                )
            )
            .whenReleased(
                new ParallelCommandGroup(
                    new InstantCommand(shooter::stopKicker),
                    new InstantCommand(indexer::stopConveyor),
                    new InstantCommand(intake::stopIntakeMotor)
                )
            );

        // toggle intake
        new JoystickButton(gamepad, Button.kX.value)
            .whenPressed(new InstantCommand(intake::toggleIntake));

        // shoot close
        new POVButton(gamepad, 0)
            .whileHeld(
                new InstantCommand(
                    () -> shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(4000))
                )
            )
            .whenReleased(new InstantCommand(shooter::stopShooter));

        // shoot mid-close
        new POVButton(gamepad, 90)
            .whileHeld(
                new InstantCommand(
                    () -> shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(5000))
                )
            )
            .whenReleased(new InstantCommand(shooter::stopShooter));

        // shoot mid-far
        new POVButton(gamepad, 180)
            .whileHeld(
                new InstantCommand(
                    () -> shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(6000))
                )
            )
            .whenReleased(new InstantCommand(shooter::stopShooter));

        // shoot far
        new POVButton(gamepad, 270)
            .whileHeld(
                new InstantCommand(
                    () -> shooter.runVelocityProfileController(Units.rotationsPerMinuteToRadiansPerSecond(7000))
                )
            )
            .whenReleased(new InstantCommand(shooter::stopShooter));

        // align with vision
        new JoystickButton(leftJoystick, Joystick.ButtonType.kTop.value)
            .whileHeld(new AlignWithTargetVision(drive, limelight));

        // quick turn to 0 degrees
        new JoystickButton(rightJoystick, Joystick.ButtonType.kTrigger.value)
            .whileHeld(new QuickTurn(drive, Math.PI));

        // quick turn to 180 degrees
        new JoystickButton(leftJoystick, Joystick.ButtonType.kTrigger.value) 
            .whileHeld(new QuickTurn(drive, 0));

        // reset imu 
        new JoystickButton(rightJoystick, 3)
            .whenPressed(new InstantCommand(drive::resetImu));

        // ramp up with vision using regression
        new JoystickButton(gamepad, Button.kX.value)
            .whileHeld(new RampUpWithVision(shooter, limelight))
            .whenReleased(new InstantCommand(() -> shooter.runShooterPercent(0)));

        // deploy climbers
        new JoystickButton(gamepad, Button.kA.value) 
            .whenPressed(new InstantCommand(climber::deployClimbers));

        // lock climbers
        new JoystickButton(gamepad, Button.kStickLeft.value)
            .whenPressed(new InstantCommand(climber::lockClimbers));

    }

    public Command getAutonomousCommand() {

        drive.resetPose(AutoTrajectories.testTrajectory.getInitialPose());
        return new FollowTrajectory(drive, AutoTrajectories.testTrajectory);

    }

}
