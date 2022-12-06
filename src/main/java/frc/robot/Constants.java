package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class CANDevices {

        public static final int frontLeftRotationMotorId = 7;
        public static final int frontLeftDriveMotorId = 8;

        public static final int frontRightRotationMotorId = 1;
        public static final int frontRightDriveMotorId = 2;

        public static final int rearLeftRotationMotorId = 6;
        public static final int rearLeftDriveMotorId = 5;

        public static final int rearRightRotationMotorId = 3;
        public static final int rearRightDriveMotorId = 4;

        public static final int leftClimberMotorId = 20;
        public static final int rightClimberMotorId = 21;

        public static final int frontLeftRotationEncoderId = 17;
        public static final int frontRightRotationEncoderId = 13;
        public static final int rearLeftRotationEncoderId = 15;
        public static final int rearRightRotationEncoderId = 14;

        public static final int flywheelMotorId = 11;

        public static final int kickerMotorId = 10;
        public static final int intakeMotorId = 16; 
        public static final int conveyorMotorId = 9;

        public static final int imuId = 18;

    }

    public static final class DIOChannels {

        public static final int bottomBannerPort = 0;
        public static final int topBannerPort = 9;

    }

    public static final class InputDevices {

        public static final int leftJoystickPort = 0;
        public static final int rightJoystickPort = 1;

        public static final int gamepadPort = 2;

    }

    public static final class PneumaticChannels {

        public  static final int PCMId = 19;

        public static final int[] intakeSolenoidChannels = {0, 1};
        public static final int lockingSolenoidChannel = 2;


    }

    public static final class DriveConstants {

        public static final double trackWidth = Units.inchesToMeters(16.5);
        public static final double wheelBase = Units.inchesToMeters(16.5);

        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //rear left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //rear right
            );

        public static final double driveWheelGearReduction = 6.86;
        public static final double rotationWheelGearReduction = 12.8;

        public static final double wheelDiameterMeters = 0.050686 * 2;

        public static final double rotationMotorMaxSpeedRadPerSec = 1.0;
        public static final double rotationMotorMaxAccelRadPerSecSq = 1.0; //TODO: Test and change max acceleration to avoid wheelies

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.254, 0.137);

        public static final double maxDriveSpeed = 14.4;
        public static final double teleopTurnRateDegPerSec = 360.0; //Rate the robot will spin with full rotation command

    }

    public static final class SuperstructureConstants {

        public static final double baseShootingSpeed = 4500; //rotations per minute

        public static final double intakingPower = 0.5;
        public static final double jogFowardPower = 0.15;

        public static final double kickerPower = 1.0;
        public static final double conveyorPower = 0.8;

        public static final double flywheelGearRatio = 32.0 / 18.0;

        public static final double jogDelaySeconds = 0.0;

    }

    public static final class ClimbingConstants {

        public static final double climberMotorGearReduction = 1;

        public static final double winchDiameterInches = 1;
        public static final double climberGearRatio = 1.0/6.0;
        public static final double climberMaxHeightInches = 40.5;

        public static final double desiredClimberSpeedInchesPerSecond = 10;

    }

    public static final class VisionConstants {
        public static final String photonCameraName = "gloworm";

        public static final Transform2d CameraToRobot = new Transform2d(new Translation2d(0.6731, 0.0), new Rotation2d(0.0));;
    }

    public static final class AutoConstants {
        public static final double maxVelMetersPerSec = 2;
        public static final double maxAccelMetersPerSecondSq = 1;
    }

    public static final class FieldConstants {
        public static final double targetHeightInches = 89.5;

        public static final Pose2d leftTagLocation = new Pose2d(new Translation2d(Units.feetToMeters(3), Units.feetToMeters(15)), Rotation2d.fromDegrees(90));
        public static final Pose2d rightTagLocation = new Pose2d(new Translation2d(Units.feetToMeters(3), Units.feetToMeters(18)), Rotation2d.fromDegrees(90));

        public static final int leftTagId = 7;
        public static final int rightTagId = 0;
    }
}
