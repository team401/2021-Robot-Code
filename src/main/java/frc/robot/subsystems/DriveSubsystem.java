package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

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

    private static final double frontLeftAngleOffset = Units.degreesToRadians(239.5 - 90);
    private static final double frontRightAngleOffset = Units.degreesToRadians(256.7 + 180 - 90);
    private static final double rearLeftAngleOffset = Units.degreesToRadians(322.8 - 90);
    private static final double rearRightAngleOffset = Units.degreesToRadians(180 + 180 - 90);

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

    }

    @Override
    public void periodic() {

        odometry.update(getHeading(), getModuleStates());
        
        SmartDashboard.putNumber("drive speed", (frontLeft.getCurrentVelocity() + frontRight.getCurrentVelocity() + rearLeft.getCurrentVelocity() + rearRight.getCurrentVelocity()) / 4);
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

        setModuleStates(states);

    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {

        frontLeft.setDesiredState(moduleStates[2]);
        frontRight.setDesiredState(moduleStates[0]);
        rearLeft.setDesiredState(moduleStates[3]);
        rearRight.setDesiredState(moduleStates[1]);

    }

    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = {
            new SwerveModuleState(frontRight.getCurrentVelocity(), frontRight.getCanEncoderAngle()),
            new SwerveModuleState(rearRight.getCurrentVelocity(), rearRight.getCanEncoderAngle()),
            new SwerveModuleState(frontLeft.getCurrentVelocity(), frontLeft.getCanEncoderAngle()),
            new SwerveModuleState(rearLeft.getCurrentVelocity(), rearLeft.getCanEncoderAngle())
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