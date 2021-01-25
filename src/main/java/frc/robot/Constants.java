package frc.robot;

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

        public static final double maxAngularVel = 1.0;
        public static final double maxAngularAccel = 1.0;


        public static final double driveKs = 0.0;
        public static final double driveKv = 0.0;
        
        public static final double rotationKs = 0.0;
        public static final double rotationKv = 0.0;

        public static final double driveGearingRatio = 1.0;
        public static final double rotationGearingRatio = 1.0;

        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0);

    }

    public static final class RobotConstants {

    }

}
