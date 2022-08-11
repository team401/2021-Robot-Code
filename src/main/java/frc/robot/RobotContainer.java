package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.drivetrain.EmptyDrive;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.commands.superstructure.indexing.Waiting;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
            .whenPressed(() -> drive.toggleDrive());

        // intake
        new JoystickButton(gamepad, Button.kB.value)
            .whenPressed(new InstantCommand(intake::runIntakeMotor))
            .whenReleased(new InstantCommand(intake::stopIntakeMotor));

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
        // spin flywheel
        new JoystickButton(rightJoystick, 2)
            .whileHeld(
                new InstantCommand(
                    () -> shooter.runVelocityProfileController
                    (Units.rotationsPerMinuteToRadiansPerSecond(2500)))
            )
            .whenReleased(new InstantCommand(shooter::stopShooter));

            new JoystickButton(rightJoystick, 4)
                .whileHeld(
                    new InstantCommand(
                        () -> shooter.runShooterPercent(0.2)
                    )
                )
                .whenReleased(
                    new InstantCommand(
                        () -> shooter.stopShooter()
                    )
                );
            
        // reset imu 
        new JoystickButton(rightJoystick, 3)
            .whenPressed(new InstantCommand(drive::resetImu));
    }

    public Command getAutonomousCommand() {

        return null;
        
    }

}
