package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.AnalogDevices;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.Utilities;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;

public class DriveSubsystem extends SubsystemBase {

    private static final double frontLeftAngleOffset = -Math.toRadians(236.5);
    private static final double frontRightAngleOffset = -Math.toRadians(54.7 + 180);
    private static final double rearLeftAngleOffset = -Math.toRadians(210.6);
    private static final double rearRightAngleOffset = -Math.toRadians(55.7 + 180);

    private final SwerveModule frontLeft = 
        new Mk2SwerveModuleBuilder(
            new Vector2(DriveConstants.trackWidth / 2.0, DriveConstants.wheelBase / 2.0))
            .angleEncoder(
                new AnalogInput(AnalogDevices.frontLeftRotationEncoderPort), frontLeftAngleOffset)
            .angleMotor(
                new CANSparkMax(CANDevices.frontLeftRotationMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                new CANSparkMax(CANDevices.frontLeftDriveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                DriveConstants.driveWheelGearReduction,
                DriveConstants.wheelDiameterMeters)
            .build();

    private final SwerveModule frontRight = 
        new Mk2SwerveModuleBuilder(
            new Vector2(DriveConstants.trackWidth / 2.0, -DriveConstants.wheelBase / 2.0))
            .angleEncoder(
                new AnalogInput(AnalogDevices.frontRightRotationEncoderPort), frontRightAngleOffset)
            .angleMotor(
                new CANSparkMax(CANDevices.frontRightRotationMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                new CANSparkMax(CANDevices.frontRightDriveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                DriveConstants.driveWheelGearReduction,
                DriveConstants.wheelDiameterMeters)
            .build();

    private final SwerveModule rearLeft = 
        new Mk2SwerveModuleBuilder(
            new Vector2(-DriveConstants.trackWidth / 2.0, DriveConstants.wheelBase / 2.0))
            .angleEncoder(
                new AnalogInput(AnalogDevices.rearLeftRotationEncoderPort), rearLeftAngleOffset)
            .angleMotor(
                new CANSparkMax(CANDevices.rearLeftRotationMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                new CANSparkMax(CANDevices.rearLeftDriveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                DriveConstants.driveWheelGearReduction,
                DriveConstants.wheelDiameterMeters)
            .build();

    private final SwerveModule rearRight = 
        new Mk2SwerveModuleBuilder(
            new Vector2(-DriveConstants.trackWidth / 2.0, -DriveConstants.wheelBase / 2.0))
            .angleEncoder(
                new AnalogInput(AnalogDevices.rearRightRotationEncoderPort), rearRightAngleOffset)
            .angleMotor(
                new CANSparkMax(CANDevices.rearRightRotationMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                new CANSparkMax(CANDevices.rearRightDriveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                DriveConstants.driveWheelGearReduction,
                DriveConstants.wheelDiameterMeters)
            .build();

    private final ADIS16470_IMU imu = new ADIS16470_IMU();

    private final SwerveDriveOdometry odometry = 
        new SwerveDriveOdometry(DriveConstants.kinematics, new Rotation2d(Units.degreesToRadians(imu.getAngle())));

    public DriveSubsystem() {

        imu.calibrate();

    }

    @Override
    public void periodic() {

        frontLeft.updateSensors();
        frontRight.updateSensors();
        rearLeft.updateSensors();
        rearRight.updateSensors();

        frontLeft.updateState(TimedRobot.kDefaultPeriod);
        frontRight.updateState(TimedRobot.kDefaultPeriod);
        rearLeft.updateState(TimedRobot.kDefaultPeriod);
        rearRight.updateState(TimedRobot.kDefaultPeriod);

        odometry.update(getHeading(), getModuleStates());

        SmartDashboard.putNumber("Gyro", getHeading().getRadians());

    }

    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

        forward = Utilities.deadband(forward);
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        strafe = Utilities.deadband(strafe);
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        rotation = Utilities.deadband(rotation);
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

        rotation *= 2.0 / Math.hypot(DriveConstants.wheelBase, DriveConstants.trackWidth);

        ChassisSpeeds speeds = (isFieldRelative)
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            forward, strafe, rotation, Rotation2d.fromDegrees(imu.getAngle()))
        : new ChassisSpeeds(forward, strafe, rotation);
        
        SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);
        
        frontLeft.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRight.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        rearLeft.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        rearRight.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());

    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {

        frontLeft.setTargetVelocity(moduleStates[0].speedMetersPerSecond, moduleStates[0].angle.getRadians());
        frontRight.setTargetVelocity(moduleStates[1].speedMetersPerSecond, moduleStates[1].angle.getRadians());
        rearLeft.setTargetVelocity(moduleStates[2].speedMetersPerSecond, moduleStates[2].angle.getRadians());
        rearRight.setTargetVelocity(moduleStates[3].speedMetersPerSecond, moduleStates[3].angle.getRadians());

    }

    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = {
            new SwerveModuleState(frontLeft.getCurrentVelocity(), new Rotation2d(frontLeft.getCurrentAngle())),
            new SwerveModuleState(frontRight.getCurrentVelocity(), new Rotation2d(frontRight.getCurrentAngle())),
            new SwerveModuleState(rearLeft.getCurrentVelocity(), new Rotation2d(rearLeft.getCurrentAngle())),
            new SwerveModuleState(rearRight.getCurrentVelocity(), new Rotation2d(rearRight.getCurrentAngle()))};

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

    public Pose2d getRobotToTarget(Pose2d targetPose) {

        Translation2d robotToTargetTranslation = 
            new Translation2d(
                targetPose.getX() - getPose().getX(), 
                targetPose.getY() - getPose().getY());

        Rotation2d robotToTargetRotation =
            new Rotation2d(
                targetPose.getRotation().getRadians() - getPose().getRotation().getDegrees()); //not sure if this'll work this way
        
        return new Pose2d(robotToTargetTranslation, robotToTargetRotation);

    }

    

}
