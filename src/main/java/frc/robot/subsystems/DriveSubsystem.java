package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOChannels;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = 
        new SwerveModule(
            CANDevices.frontLeftDriveSparkId, 
            CANDevices.frontLeftRotationSparkId, 
            DIOChannels.frontLeftRotationEncoderPort);

    private final SwerveModule frontRight = 
        new SwerveModule(
            CANDevices.frontRightDriveSparkId, 
            CANDevices.frontRightRotationSparkId, 
            DIOChannels.frontRightRotationEncoderPort);

    private final SwerveModule rearLeft = 
        new SwerveModule(
            CANDevices.rearLeftDriveSparkId, 
            CANDevices.rearLeftRotationSparkId, 
            DIOChannels.rearLeftRotationEncoderPort);

    private final SwerveModule rearRight = 
        new SwerveModule(
            CANDevices.rearRightDriveSparkId, 
            CANDevices.rearRightRotationSparkId, 
            DIOChannels.rearRightRotationEncoderPort);

    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    private final SwerveDriveKinematics kinematics = 
        new SwerveDriveKinematics(
            DriveConstants.frontLeftModuleLocation,
            DriveConstants.frontRightModuleLocation,
            DriveConstants.rearLeftModuleLocation,
            DriveConstants.rearRightModuleLocation);

    private final SwerveDriveOdometry odometry = 
        new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

    public DriveSubsystem() {

        gyro.reset();

    }

    public void drive(double xSpeed, double ySpeed, double rot) {

        var swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.maxSpeedMetersPerSecond);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);

    }

    public void updateOdometry() {

        odometry.update(
            gyro.getRotation2d(),
            frontLeft.getState(),
            frontRight.getState(),
            rearLeft.getState(),
            rearRight.getState());

    }

}
