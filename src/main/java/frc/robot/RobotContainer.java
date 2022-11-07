package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.InputDevices;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Chase;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.commands.superstructure.indexing.Waiting;
import frc.robot.commands.superstructure.shooting.RampUpToSpeed;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TagVisionSubsystem;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

    private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final XboxController gamepad = new XboxController(InputDevices.gamepadPort);

    private final DriveSubsystem drive = new DriveSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final IndexingSubsystem indexer = new IndexingSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final TagVisionSubsystem vision = new TagVisionSubsystem(
        new PhotonCamera(VisionConstants.photonCameraName), drive);
    
    public RobotContainer() {
        drive.setDefaultCommand(
            new OperatorControl(
                drive,
                () -> leftJoystick.getY(),
                () -> leftJoystick.getX(),
                () -> rightJoystick.getX(),
                true
            )
        );
        indexer.setDefaultCommand(new Waiting(indexer));
        shooter.setDefaultCommand(
            new RunCommand(
                () -> shooter.runShooterPercent(gamepad.getRightTriggerAxis()), 
                shooter
            )
        );

        configureButtonBindings();
    }

    public void configureButtonBindings() {
        // Toggle driving
        new JoystickButton(gamepad, Button.kStart.value)
            .whenPressed(() -> drive.toggleDrive(true))
            .whenReleased(() -> drive.toggleDrive(false));
        // intake
        new JoystickButton(gamepad, Button.kB.value)
            .whenPressed(intake::runIntakeMotor)
            .whenReleased(intake::stopIntakeMotor);

        // shoot
        new JoystickButton(gamepad, Button.kY.value)
            .whileHeld(
                new InstantCommand(shooter::runKicker)
                .alongWith(new InstantCommand(indexer::runConveyor, indexer))
            )
            .whenReleased(
                new InstantCommand(shooter::stopKicker)
                .alongWith(new InstantCommand(indexer::stopConveyor, indexer))
            );
        // spin flywheel
        /*new JoystickButton(rightJoystick, 2)
        .whileHeld(
            () -> shooter.runVelocityProfileController
            (Units.rotationsPerMinuteToRadiansPerSecond(2500)), shooter)
        .whenReleased(shooter::stopShooter);
        // auto-shoot
        new JoystickButton(leftJoystick, 2)
        .whenPressed(new RampUpToSpeed(Units.rotationsPerMinuteToRadiansPerSecond(2500), shooter, indexer));*/
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
                    new InstantCommand(indexer::stopConveyor, indexer),
                    new InstantCommand(intake::stopIntakeMotor)
                )
            );
        // Follow AprilTag, UNTESTED
        new JoystickButton(gamepad, Button.kX.value)
            .whileHeld(new Chase(drive, vision));
        // reset realitive forward
        new JoystickButton(rightJoystick, 3)
            .whenPressed(drive::resetRealitivity);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}