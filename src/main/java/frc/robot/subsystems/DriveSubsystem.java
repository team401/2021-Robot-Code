package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.AnalogDevices;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private static final double frontLeftAngleOffset = -Math.toRadians(236.5);
    private static final double frontRightAngleOffset = -Math.toRadians(54.7 + 180);
    private static final double rearLeftAngleOffset = -Math.toRadians(210.6);
    private static final double rearRightAngleOffset = -Math.toRadians(55.7 + 180);

        private final SwerveModule frontLeft = 
            new SwerveModule(
                CANDevices.frontLeftDriveMotorId,
                CANDevices.frontLeftRotationMotorId,
                AnalogDevices.frontLeftRotationEncoderPort,
                frontLeftAngleOffset
            );

        private final SwerveModule frontRight = 
            new SwerveModule(
                CANDevices.frontRightDriveMotorId,
                CANDevices.frontRightRotationMotorId,
                AnalogDevices.frontRightRotationEncoderPort,
                frontRightAngleOffset
            );

        private final SwerveModule rearLeft = 
            new SwerveModule(
                CANDevices.rearLeftDriveMotorId,
                CANDevices.rearLeftRotationMotorId,
                AnalogDevices.rearLeftRotationEncoderPort,
                rearLeftAngleOffset
            );
        private final SwerveModule rearRight = 
            new SwerveModule(
                CANDevices.rearRightDriveMotorId,
                CANDevices.rearRightRotationMotorId,
                AnalogDevices.rearRightRotationEncoderPort,
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

        SmartDashboard.putNumber("front left integrated encoder angle reading degrees", frontLeft.getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("front left external encoder angle reading degrees", frontLeft.getAnalogEncoderAngle().getDegrees());
        
    }

    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

        rotation *= 2.0 / Math.hypot(DriveConstants.wheelBase, DriveConstants.trackWidth);

        ChassisSpeeds speeds = (isFieldRelative)

            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                forward, strafe, rotation, Rotation2d.fromDegrees(imu.getAngle())
            )
            : new ChassisSpeeds(forward, strafe, rotation);
        
        SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        rearLeft.setDesiredState(states[2]);
        rearRight.setDesiredState(states[3]);

    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {

        /*frontLeft.setTargetVelocity(moduleStates[0].speedMetersPerSecond, moduleStates[0].angle.getRadians());
        frontRight.setTargetVelocity(moduleStates[1].speedMetersPerSecond, moduleStates[1].angle.getRadians());
        rearLeft.setTargetVelocity(moduleStates[2].speedMetersPerSecond, moduleStates[2].angle.getRadians());
        rearRight.setTargetVelocity(moduleStates[3].speedMetersPerSecond, moduleStates[3].angle.getRadians());*/

    }

    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = {
            new SwerveModuleState(frontLeft.getCurrentVelocity(), frontLeft.getCurrentAngle()),
            new SwerveModuleState(frontRight.getCurrentVelocity(), frontRight.getCurrentAngle()),
            new SwerveModuleState(rearLeft.getCurrentVelocity(), rearLeft.getCurrentAngle()),
            new SwerveModuleState(rearRight.getCurrentVelocity(), rearRight.getCurrentAngle())
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

    /*public Pose2d getRobotToTarget(Pose2d targetPose) {

        Pose2d robotToTargetPose = 
            new Pose2d(
                targetPose.getX() - getPose().getX(), 
                targetPose.getY() - getPose().getY()
            );

        Rotation2d robotToTargetRotation =
            //not sure if this'll work this way
            new Rotation2d(
                targetPose.getRotation().getRadians() - getPose().getRotation().getDegrees()); 
        
        return new Pose2d(robotToTargetPose, robotToTargetRotation);

    }*/

}