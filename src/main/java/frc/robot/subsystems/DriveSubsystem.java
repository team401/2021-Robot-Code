package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private static final double frontLeftAngleOffset = Units.degreesToRadians(326.78 + 180);
    private static final double frontRightAngleOffset = Units.degreesToRadians(344.27 + 180);
    private static final double rearLeftAngleOffset = Units.degreesToRadians(50.00);
    private static final double rearRightAngleOffset = Units.degreesToRadians(266.13);

        private final SwerveModule frontLeft = 
            new SwerveModule(
                CANDevices.frontLeftDriveMotorId,
                CANDevices.frontLeftRotationMotorId,
                CANDevices.frontLeftRotationEncoderId,
                frontLeftAngleOffset
            );

        private final SwerveModule frontRight = 
            new SwerveModule(
                CANDevices.frontRightDriveMotorId,
                CANDevices.frontRightRotationMotorId,
                CANDevices.frontRightRotationEncoderId,
                frontRightAngleOffset
            );

        private final SwerveModule rearLeft = 
            new SwerveModule(
                CANDevices.rearLeftDriveMotorId,
                CANDevices.rearLeftRotationMotorId,
                CANDevices.rearLeftRotationEncoderId,
                rearLeftAngleOffset
            );

        private final SwerveModule rearRight = 
            new SwerveModule(
                CANDevices.rearRightDriveMotorId,
                CANDevices.rearRightRotationMotorId,
                CANDevices.rearRightRotationEncoderId,
                rearRightAngleOffset
            );

    private final ADIS16470_IMU imu = new ADIS16470_IMU();

    private final SwerveDriveOdometry odometry = 
        new SwerveDriveOdometry(
            DriveConstants.kinematics, 
            new Rotation2d(Units.degreesToRadians(imu.getAngle()))
        );

    public DriveSubsystem() {

        imu.calibrate();

        frontLeft.initRotationMotorOffset();
        frontRight.initRotationMotorOffset();
        rearLeft.initRotationMotorOffset();
        rearRight.initRotationMotorOffset();

    }

    @Override
    public void periodic() {

        odometry.update(getHeading(), getModuleStates());

        SmartDashboard.putNumber("front Left canCoder", frontLeft.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("front Right canCoder", frontRight.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("rear Left canCoder", rearLeft.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("rear Right canCoder", rearRight.getCanCoderAngle().getDegrees());

        SmartDashboard.putNumber("front Left canEncoder", frontLeft.getCanEncoderAngle().getDegrees());
        SmartDashboard.putNumber("front Right canEncoder", frontRight.getCanEncoderAngle().getDegrees());
        SmartDashboard.putNumber("rear Left canEncoder", rearLeft.getCanEncoderAngle().getDegrees());
        SmartDashboard.putNumber("rear Right canEncoder", rearRight.getCanEncoderAngle().getDegrees());

    }

    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

        ChassisSpeeds speeds = 
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    forward, strafe, rotation, Rotation2d.fromDegrees(imu.getAngle()))
                : new ChassisSpeeds(forward, strafe, rotation);
        
        SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);

        setModuleStates(states);

    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {

        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        rearLeft.setDesiredState(moduleStates[2]);
        rearRight.setDesiredState(moduleStates[3]);

    }

    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = {
            new SwerveModuleState(frontLeft.getCurrentVelocity(), frontLeft.getCanCoderAngle()),
            new SwerveModuleState(frontRight.getCurrentVelocity(), frontRight.getCanCoderAngle()),
            new SwerveModuleState(rearLeft.getCurrentVelocity(), rearLeft.getCanCoderAngle()),
            new SwerveModuleState(rearRight.getCurrentVelocity(), rearRight.getCanCoderAngle())
        };

        return states;

    }

    public Pose2d getPose() {
        
        return odometry.getPoseMeters();

    }

    public void resetPose(Pose2d pose) {

        imu.reset();
        new Rotation2d();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(-imu.getAngle()));

    }

    public Rotation2d getHeading() {

        new Rotation2d();
        return Rotation2d.fromDegrees(-imu.getAngle());

    }

}