package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    public static final class CANDevices {

        public static final int frontLeftRotationMotorId = 2;
        public static final int frontLeftDriveMotorId = 1;

        public static final int frontRightRotationMotorId = 4;
        public static final int frontRightDriveMotorId = 3;

        public static final int rearLeftRotationMotorId = 6;
        public static final int rearLeftDriveMotorId = 5;

        public static final int rearRightRotationMotorId = 8;
        public static final int rearRightDriveMotorId = 7;

        public static final int rightFlywheelMotorId = 9;
        public static final int leftFlywheelMotorId = 10;

        public static final int kickerMotorId = 11;
        public static final int intakeMotorId = 12;
        public static final int conveyorMotorId = 13;

    }

    public static final class AnalogDevices {

        public static final int frontLeftRotationEncoderPort = 1;
        public static final int frontRightRotationEncoderPort = 0;
        public static final int rearLeftRotationEncoderPort = 2;
        public static final int rearRightRotationEncoderPort = 3;

    }

    public static final class DIOChannels {

        public static final int bottomBannerPort = 0;
        public static final int topBannerPort = 1;

    }

    public static final class InputDevices {

        public static final int leftJoystickPort = 0;
        public static final int rightJoystickPort = 1;

        public static final int gamepadPort = 2;

    }

    public static final class PneumaticChannels {

        public static final int[] leftIntakeSolenoidChannels = {0, 1};
        public static final int[] rightIntakeSolenoidChannels = {2, 3};
        public static final int[] hoodSolenoidChannels = {4, 5};

    }

    public static final class DriveConstants {

        public static final double trackWidth = Units.inchesToMeters(16.5);
        public static final double wheelBase = Units.inchesToMeters(16.5);

        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0));

        public static final double driveWheelGearReduction = 8.33;
        public static final double rotationWheelGearReduction = 19.0;

        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0);

    }

    public static final class SuperstructureConstants {

        public static final double intakingPower = 0.5;
        public static final double kickerPower = 0.85;
        public static final double conveyorPower = 0.5;

        public static final double spacingDelaySeconds = 0.5;

        public static final double flywheelGearRatio = 1.0 / 1.0;

    }

    public static final class AutoConstants {

        public static final double maxVelMetersPerSec = 1;
        public static final double maxAccelMetersPerSecondSq = .5;

        public static final double maxAngularSpeedRadPerSec = 5;
        public static final double maxAngularAccelRadPerSecSq = 5;

    }

    public static final class FieldConstants {

        public static final Pose2d initalRobotToFieldPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

    }
    
}
