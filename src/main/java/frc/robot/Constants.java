package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

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

        public static final int frontLeftRotationEncoderId = 17;
        public static final int frontRightRotationEncoderId = 13;
        public static final int rearLeftRotationEncoderId = 15;
        public static final int rearRightRotationEncoderId = 14;

        public static final int rightFlywheelMotorId = 12;
        public static final int leftFlywheelMotorId = 11;

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

        //public static final int[] intakeSolenoidChannels = {0, 1};

    }

    public static final class DriveConstants {

        public static final double trackWidth = Units.inchesToMeters(16.5);
        public static final double wheelBase = Units.inchesToMeters(16.5);

        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
            );

        public static final double driveWheelGearReduction = 6.86;
        public static final double rotationWheelGearReduction = 12.8;

        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0);

        public static final double rotationMotorMaxSpeedRadPerSec = 1.0;
        public static final double rotationMotorMaxAccelRadPerSecSq = 1.0;

        public static final double maxDriveSpeed = 3.75; // meters per second

    }

    public static final class SuperstructureConstants {

        public static final double baseShootingSpeed = 6000;

        public static final double intakingPower = 0.6;
        public static final double jogFowardPower = 0.2;

        public static final double kickerPower = 1.0;
        public static final double conveyorPower = 0.35;

        public static final double flywheelGearRatio = 32.0 / 18.0;

        public static final double jogDelaySeconds = 0.1;

    }

    public static final class AutoConstants {

        public static final double maxVelMetersPerSec = 1.5;
        public static final double maxAccelMetersPerSecondSq = 1;

        public static final double maxAngularSpeedRadPerSec = 5;
        public static final double maxAngularAccelRadPerSecSq = 5;
        
    }

    public static final class FieldConstants {

        public static final Pose2d initalRobotToFieldPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

    }
    
}
