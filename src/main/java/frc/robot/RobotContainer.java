package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final static XboxController gamepad = new XboxController(InputDevices.gamepadPort);

    private DriveSubsystem drive = new DriveSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ConveyorSubsystem conveyor = new ConveyorSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    
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

        intake.compressorPSI();

    }

    public void configureButtonBindings() {

        // Collects balls 
        new JoystickButton(gamepad, Button.kB.value)

            .whileHeld(intake::runIntakeMotor, intake)    
            .whileHeld(conveyor::runConveyor, conveyor);
        
        // Rev up shooter
     /*
        new JoystickButton(gamepad, Button.kB.value)
            
            .whileHeld(shooter::runShooterPercent, shooter);           
    */
        new JoystickButton(gamepad, Axis.kLeftTrigger.value)
        
            .whileHeld(shooter::runShooterPercent, shooter); 
            
        // Shoots balls
        new JoystickButton(gamepad, Axis.kRightTrigger.value)
        
            .whileHeld(shooter::runKicker, shooter)
            .whileHeld(conveyor::runConveyor, conveyor); 
    
    }

    public Command getAutonomousCommand() {

        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.maxVelMetersPerSec, 
            AutoConstants.maxAccelMetersPerSecondSq
        )
        .setKinematics(DriveConstants.kinematics);

        config.setStartVelocity(0.0);
        config.setEndVelocity(0.0);
/*
        String trajectoryJSON = "paths/Test.wpilib.json";
    
        Trajectory trajectory = new Trajectory();
    
        try {

            Path trajpath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            if (Files.exists(trajpath)){
                trajectory = TrajectoryUtil.fromPathweaverJson(trajpath);
                SmartDashboard.putString("yes", "yup file"); 
            } else {
                SmartDashboard.putString("none", "no file"); 
            }

        } catch(IOException ex) {

        }
*/
        Trajectory autoNavSlalomTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(48.5), Units.inchesToMeters(30), new Rotation2d(0)),
            List.of(new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(60)),
                    new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(90)),
                    new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(90)),
                    new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(60)),
                    new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(30)),
                    new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(60)),
                    new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(90)),
                    new Translation2d(Units.inchesToMeters(280), Units.inchesToMeters(60)),
                    new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(20)),
                    new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(20)),
                    new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(55))),
            new Pose2d(Units.inchesToMeters(48.5), Units.inchesToMeters(75), new Rotation2d(0)), config);

        FollowTrajectory runTrajectory = new FollowTrajectory(drive, autoNavSlalomTrajectory);

        //drive.resetPose(runTrajectory.getInitialPose());

        return runTrajectory;

    }
    
    public static final class Gamepad {
       
        public static double getGamepadTrigger() {

            return gamepad.getTriggerAxis(Hand.kLeft);
    
        }

    }
}
