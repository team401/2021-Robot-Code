package frc.robot;

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

    }

    public static final class AnalogDevices {

        public static final int frontLeftRotationEncoderPort = 0;
        public static final int frontRightRotationEncoderPort = 1;
        public static final int rearLeftRotationEncoderPort = 2;
        public static final int rearRightRotationEncoderPort = 3;

    }

    public static final class InputDevices {

        public static final int leftJoystickPort = 0;
        public static final int rightJoystickPort = 1;

    }

    public static final class PneumaticChannels {

        public static final int[] leftIntakeSolenoidChannels = {0, 1};
        public static final int[] rightIntakeSolenoidChannels= {2, 3};

    }
    
}
