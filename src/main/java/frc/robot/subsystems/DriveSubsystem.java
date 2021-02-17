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
                Mk2SwerveModuleBuilder.MotorType.NEO)
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
                Mk2SwerveModuleBuilder.MotorType.NEO)
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
                Mk2SwerveModuleBuilder.MotorType.NEO)
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
                Mk2SwerveModuleBuilder.MotorType.NEO)
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

        SmartDashboard.putNumber("Front Left Module Angle", Math.toDegrees(frontLeft.getCurrentAngle()));
        SmartDashboard.putNumber("Front Right Module Angle", Math.toDegrees(frontRight.getCurrentAngle()));
        SmartDashboard.putNumber("Back Left Module Angle", Math.toDegrees(rearLeft.getCurrentAngle()));
        SmartDashboard.putNumber("Back Right Module Angle", Math.toDegrees(rearRight.getCurrentAngle()));

        frontLeft.updateState(TimedRobot.kDefaultPeriod);
        frontRight.updateState(TimedRobot.kDefaultPeriod);
        rearLeft.updateState(TimedRobot.kDefaultPeriod);
        rearRight.updateState(TimedRobot.kDefaultPeriod);

    }

    public void drive(Translation2d translation, double rotation, boolean isFieldRelative) {

        rotation *= 2.0 / Math.hypot(DriveConstants.wheelBase, DriveConstants.trackWidth);

        ChassisSpeeds speeds = (isFieldRelative)
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, Rotation2d.fromDegrees(imu.getAngle()))
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        
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

    public Pose2d getPose() {

        return odometry.getPoseMeters();

    }

}
