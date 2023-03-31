// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class CANDevices {

        public static final int frontLeftDriveMotorID = 8;
        public static final int frontLeftRotationMotorID = 7;

        public static final int frontRightDriveMotorID = 2;
        public static final int frontRightRotationMotorID = 1;

        public static final int backLeftDriveMotorID = 5;
        public static final int backLeftRotationMotorID = 6;

        public static final int backRightDriveMotorID = 4;
        public static final int backRightRotationMotorID = 3;

        public static final int frontLeftRotationEncoderID = 17;
        public static final int frontRightRotationEncoderID = 13;
        public static final int backLeftRotationEncoderID = 15;
        public static final int backRightRotationEncoderID = 14;

        public static final int flywheelMotorId = 11;

        public static final int kickerMotorId = 10;
        public static final int intakeMotorId = 16; 
        public static final int conveyorMotorId = 9;

        public static final int pigeonIMU = 18;

    }

    public static final class DIOChannels {

        public static final int bottomBannerPort = 0;
        public static final int topBannerPort = 9;

    }
    
    public static final class DriveConstants {

        public static final double trackWidth = Units.inchesToMeters(16.5);
        public static final double wheelBase = Units.inchesToMeters(16.5);
        public static final double wheelRadiusM = 0.050686;

        public static final double driveWheelGearReduction = 6.86;
        public static final double rotationWheelGearReduction = 12.8;

        public static final double frontLeftAngleOffset = 5.7125;
        public static final double frontRightAngleOffset = 2.847;
        public static final double backLeftAngleOffset = 0.87;
        public static final double backRightAngleOffset = 1.546;

        public static final double driveKp = 0.01;
        public static final double driveKd = 0.0;
        public static final double rotationKp = 1;
        public static final double rotationKd = 0.5;

        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //rear left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //rear right
        );

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.254, 0.137);

        public static final double maxDriveSpeed = 3;
        public static final double maxTurnRate = Math.PI;

        public static final double driveJoystickDeadbandPercent = 0.075;
        public static final double driveMaxJerk = 100.0;

    }

    public static final class SuperstructureConstants {

        public static final double baseShootingSpeed = 4500;

        public static final double intakingPower = 0.5;
        public static final double jogFowardPower = 0.15;

        public static final double kickerPower = 1.0;
        public static final double conveyorPower = 0.8;

        public static final double flywheelGearRatio = 32.0 / 18.0;

        public static final double jogDelaySeconds = 0.0;

    }

}