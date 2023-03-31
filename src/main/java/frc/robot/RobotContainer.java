// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.index.Waiting;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;

public class RobotContainer {
    
    Drive drive = new Drive();
    Index index = new Index();
    Intake intake = new Intake();
    Shooter shooter = new Shooter();

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);

    public RobotContainer() {

        configureSubsystems();
        configureBindings();

    }

    private void configureSubsystems() {

        drive.setDefaultCommand(new DriveWithJoysticks(
            drive,
            () -> -leftStick.getRawAxis(1),
            () -> -leftStick.getRawAxis(0),
            () -> -rightStick.getRawAxis(0),
            true
        ));
        SmartDashboard.putNumber("Controller Reduction", 1);

        //index.setDefaultCommand(new Waiting(index));
        shooter.setDefaultCommand(
            new RunCommand(() -> shooter.runShooterPercent(gamepad.getRightTriggerAxis()), shooter)
        );

    }

    private void configureBindings() {
        // Reset Gyro
        new Trigger(() -> rightStick.getRawButtonPressed(2))
            .onTrue(new InstantCommand(drive::resetHeading));
        
        // Intake
        new JoystickButton(gamepad, Button.kB.value)
            .onTrue(new InstantCommand(intake::runIntakeMotor))
            .onFalse(new InstantCommand(intake::stopIntakeMotor));

        // Shoot
        new JoystickButton(gamepad, Button.kY.value)
            .onTrue(new InstantCommand(shooter::runKicker)
                .alongWith(new InstantCommand(index::runConveyor, index)))
            .onFalse(new InstantCommand(shooter::stopKicker)
                .alongWith(new InstantCommand(index::stopConveyor, index)));
        

        // manual reverse
        new JoystickButton(gamepad, Button.kBack.value)
            .onTrue(new ParallelCommandGroup(
                new InstantCommand(shooter::reverseKicker),
                new InstantCommand(index::reverseConveyor, index),
                new InstantCommand(intake::reverseIntakeMotor)
            ))
            .onFalse(new ParallelCommandGroup(
                new InstantCommand(shooter::stopKicker),
                new InstantCommand(index::stopConveyor, index),
                new InstantCommand(intake::stopIntakeMotor)
            ));
        
    }

    public Command getAutonomousCommand() {
        return null;
    }   
}
