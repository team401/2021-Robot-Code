package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

public final class Constants {

    public static final class CANDevices {

        public static final int frontLeftDriveSparkId = 2;
        public static final int frontLeftRotationSparkId = 1;

        public static final int frontRightDriveSparkId = 4;
        public static final int frontRightRotationSparkId = 5;

        public static final int rearLeftDriveSparkId = 8;
        public static final int rearLeftRotationSparkId = 3;

        public static final int rearRightDriveSparkId = 6;
        public static final int rearRightRotationSparkId = 7;

    }

    public static final class ModuleConstants {
    
        public static final double driveControllerKp = 0.0;
        public static final double driveControllerKi = 0.0;
        public static final double driveControllerKd = 0.0;

        public static final double rotationControllerKp = 0.0;
        public static final double rotationControllerKi = 0.0;
        public static final double rotationControllerKd = 0.0;

        public static final double maxAngularVelRadPerSec = 1.0;
        public static final double maxAngularAccelRadPerSecSq = 1.0;

        public static final double driveKs = 0.0;
        public static final double driveKv = 0.0;
        
        public static final double rotationKs = 0.0;
        public static final double rotationKv = 0.0;

        public static final double driveGearingRatio = 8.33 / 1.0; 
        public static final double rotationGearingRatio = 18.0 / 1.0;

        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0);

    }

    public static final class DIOChannels {

        public static final int frontLeftRotationEncoderPort = 1;
        public static final int frontRightRotationEncoderPort = 0;
        public static final int rearLeftRotationEncoderPort = 2;
        public static final int rearRightRotationEncoderPort = 3;

    }

    public static final class DriveConstants {

        public static final double maxSpeedMetersPerSecond = 10.0;

        public static final double wheelBaseMeters = Units.inchesToMeters(16.5);
        public static final double trackWidthMeters = Units.inchesToMeters(16.5);

        public static final Translation2d frontLeftModuleLocation = new Translation2d(-trackWidthMeters / 2, wheelBaseMeters / 2);
        public static final Translation2d frontRightModuleLocation = new Translation2d(trackWidthMeters / 2, wheelBaseMeters / 2);
        public static final Translation2d rearLeftModuleLocation = new Translation2d(-trackWidthMeters / 2, -wheelBaseMeters / 2);
        public static final Translation2d rearRightModuleLocation = new Translation2d(trackWidthMeters / 2, -wheelBaseMeters / 2);

    }

    public static final class InputDevices {

        public static final int leftJoystickPort = 0;
        public static final int rightJoystickPort = 1;

    }

}
