package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.drivetrain.AlignWithTargetVision;
import frc.robot.commands.drivetrain.OperatorControl;
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

        configureButtonBindings();
        
        drive.setDefaultCommand(
            new OperatorControl(
                drive, 
                () -> leftJoystick.getX(GenericHID.Hand.kLeft), 
                () -> leftJoystick.getY(GenericHID.Hand.kLeft), 
                () -> rightJoystick.getX(GenericHID.Hand.kRight),
                true
            )
        );

        indexer.setDefaultCommand(
            new Waiting(indexer)
        );

    }

    public void configureButtonBindings() {

        // intake
        new JoystickButton(gamepad, Button.kB.value)
            .whenPressed(new InstantCommand(intake::runIntakeMotor))
            .whenReleased(new InstantCommand(intake::stopIntakeMotor));
        
        // ramp up shooter
        new JoystickButton(gamepad, Button.kBumperRight.value)
            .whenPressed(new RampUpWithVision(shooter, limelight))
            .whenReleased(new InstantCommand(shooter::stopShooter));

        // shoot
        new JoystickButton(gamepad, Button.kY.value)
            .whenPressed(
                new ConditionalCommand(
                    new InstantCommand(indexer::runConveyor)
                    .alongWith(new InstantCommand(shooter::runKicker)), 
                    new InstantCommand(), // maybe has to have stopConveyor and stopKicker?
                shooter::atGoal
                )
            )
            .whenReleased(
                new InstantCommand(shooter::stopKicker)
                .alongWith(new InstantCommand(indexer::stopConveyor))
            );

        // manual reverse
        new JoystickButton(gamepad, Button.kBack.value) 
            .whenPressed(
                new ParallelCommandGroup(
                    new InstantCommand(shooter::reverseKicker),
                    new InstantCommand(indexer::reverseConveyor),
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

        // align with vision
        new JoystickButton(leftJoystick, Joystick.ButtonType.kTop.value)

            .whileHeld(new AlignWithTargetVision(drive, limelight));

    }

    /*public Command getAutonomousCommand() {

        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.maxVelMetersPerSec, 
            AutoConstants.maxAccelMetersPerSecondSq
        )
        .setKinematics(DriveConstants.kinematics);

        config.setStartVelocity(0.0);
        config.setEndVelocity(0.0);

        FollowTrajectory runTrajectory = new FollowTrajectory(drive, AutoTrajectories.autoNavBarrelTrajectory);

        drive.resetPose(runTrajectory.getInitialPose());

        return new FollowTrajectory(drive, AutoTrajectories.autoNavBarrelTrajectory);

    }*/

}
