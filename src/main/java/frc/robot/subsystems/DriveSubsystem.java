package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private static final double frontLeftAngleOffset = Units.degreesToRadians(239.5);
    private static final double frontRightAngleOffset = Units.degreesToRadians(256.7 + 180);
    private static final double rearLeftAngleOffset = Units.degreesToRadians(322.8);
    private static final double rearRightAngleOffset = Units.degreesToRadians(180.0 + 180);

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

    private double commandedForward = 0;
    private double commandedStrafe = 0;
    private double commandedRotation = 0;

    private boolean isCommandedFieldRelative = false;

    private final PigeonIMU imu = new PigeonIMU(CANDevices.imuId);

    private final SwerveDriveOdometry odometry = 
        new SwerveDriveOdometry(
            DriveConstants.kinematics, 
            new Rotation2d(getHeading().getRadians())
        );

    public DriveSubsystem() {

        imu.setYaw(0.0);

        frontLeft.initRotationOffset();
        frontRight.initRotationOffset();
        rearLeft.initRotationOffset();
        rearRight.initRotationOffset();

        frontLeft.resetDistance();
        frontRight.resetDistance();
        rearLeft.resetDistance();
        rearRight.resetDistance();

    }

    @Override
    public void periodic() {

        odometry.update(getHeading(), getModuleStates());
        
    }
    
    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

        commandedForward = forward;
        commandedStrafe = strafe;
        commandedRotation = rotation;

        isCommandedFieldRelative = isFieldRelative;

        ChassisSpeeds speeds =
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    forward, strafe, rotation, getHeading())
                : new ChassisSpeeds(forward, strafe, rotation);
        
        SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.normalizeWheelSpeeds(states, DriveConstants.maxDriveSpeed);

        setModuleStates(states);

    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {

        frontLeft.setDesiredStateClosedLoop(moduleStates[0]);
        frontRight.setDesiredStateClosedLoop(moduleStates[1]);
        rearLeft.setDesiredStateClosedLoop(moduleStates[2]);
        rearRight.setDesiredStateClosedLoop(moduleStates[3]);

    }

    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = {
            new SwerveModuleState(frontRight.getCurrentVelocityMetersPerSecond(), frontRight.getCanEncoderAngle()),
            new SwerveModuleState(rearRight.getCurrentVelocityMetersPerSecond(), rearRight.getCanEncoderAngle()),
            new SwerveModuleState(frontLeft.getCurrentVelocityMetersPerSecond(), frontLeft.getCanEncoderAngle()),
            new SwerveModuleState(rearLeft.getCurrentVelocityMetersPerSecond(), rearLeft.getCanEncoderAngle())
        };

        return states;

    }

    public Pose2d getPose() {

        return odometry.getPoseMeters();

    }

    public void resetPose(Pose2d pose) {

        imu.setYaw(0);
        odometry.resetPosition(pose, getHeading());

    }

    public void resetDriveDistances() {

        frontLeft.resetDistance();
        frontRight.resetDistance();
        rearLeft.resetDistance();
        rearRight.resetDistance();

    }

    public double getAverageDriveDistanceRadians() {

        return ((
            Math.abs(frontLeft.getDriveDistanceRadians())
            + Math.abs(frontRight.getDriveDistanceRadians())
            + Math.abs(rearLeft.getDriveDistanceRadians())
            + Math.abs(rearRight.getDriveDistanceRadians())) / 4.0);

    }

    public double getAverageDriveVelocityRadiansPerSecond() {

        return ((
            Math.abs(frontLeft.getCurrentVelocityRadiansPerSecond())
            + Math.abs(frontRight.getCurrentVelocityRadiansPerSecond()) 
            + Math.abs(rearLeft.getCurrentVelocityRadiansPerSecond()) 
            + Math.abs(rearRight.getCurrentVelocityRadiansPerSecond())) / 4.0);

    }

    public Rotation2d getHeading() {

        double[] ypr = new double[3];

        imu.getYawPitchRoll(ypr);

        return Rotation2d.fromDegrees(ypr[0]);

    }

    public double[] getCommandedDriveValues() {

        double[] values = {commandedForward, commandedStrafe, commandedRotation};

        return values;

    }

    public boolean getIsFieldRelative() {

        return isCommandedFieldRelative;

    }

    public void resetImu() {

        imu.setYaw(0);

    }

}