package frc.robot;

import java.nio.BufferOverflowException;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.drivetrain.AlignWithTargetVision;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.commands.superstructure.ShootWithVision;
import frc.robot.commands.superstructure.Indexing.Waiting;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final static XboxController gamepad = new XboxController(InputDevices.gamepadPort);

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
                false
            )
        );

        //intake.setDefaultCommand(new InstantCommand(intake::stopIntakeMotor, intake));

        indexer.setDefaultCommand(
            new Waiting(indexer)
        );

        //shooter.setDefaultCommand(new InstantCommand(shooter::stop, shooter));

    }

    public void configureButtonBindings() {

        // intake
        new JoystickButton(gamepad, Button.kB.value)

            .whileHeld(intake::runIntakeMotor, intake)

            .whenReleased(intake::stopIntakeMotor, intake);
        
        // ramp up shooter with vision
        new JoystickButton(gamepad, Button.kBumperRight.value)

            .whileHeld(new ShootWithVision(shooter, limelight))

            .whenReleased(shooter::stopShooter, shooter);

        // spin up for manual control
        new JoystickButton(gamepad, Button.kBumperLeft.value)
            
            .whileHeld(() -> shooter.runShooterPercent(0.4))

            .whenReleased(shooter::stopShooter, shooter);

        // spin up for far position  
        /*new JoystickButton(gamepad, )

            .whileHeld(() -> shooter.runShooterPercent(0.75), shooter);

        // spin up for mid position
        new JoystickButton(gamepad, Button.kBumperLeft.value) 

            .whileHeld(() -> shooter.runShooterPercent(0.5), shooter);

        // spin up for close position
        new JoystickButton(gamepad, Button.kBumperLeft.value) 

            .whileHeld(() -> shooter.runShooterPercent(0.25), shooter);*/

        
        // shoot a ball
        new JoystickButton(gamepad, Button.kY.value)
        
            .whileHeld(shooter::runKicker, shooter)
            .whileHeld(indexer::runConveyor, indexer)
            
            .whenReleased(shooter::stopKicker, shooter)
            .whenReleased(indexer::stopConveyor, indexer);

        // manual reverse
        new JoystickButton(gamepad, Button.kBack.value) 

            .whileHeld(shooter::reverseKicker, shooter)
            .whileHeld(indexer::reverseConveyor, indexer)
            .whileHeld(intake::reverseIntakeMotor, intake)

            .whenReleased(shooter::stopKicker, shooter)
            .whenReleased(indexer::stopConveyor, indexer)
            .whenReleased(intake::stopIntakeMotor, intake);

        // align with vision
        new JoystickButton(leftJoystick, Joystick.ButtonType.kTop.value)

            .whileHeld(new AlignWithTargetVision(drive, limelight));



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

        drive.resetPose(runTrajectory.getInitialPose());

        return new FollowTrajectory(drive, AutoTrajectories.autoNavBarrelTrajectory);

    }

}
