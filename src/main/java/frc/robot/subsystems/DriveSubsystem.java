package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants.AnalogDevices;
import frc.robot.Constants.CANDevices;

import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;

public class DriveSubsystem extends SubsystemBase {

    private static final double trackWidth = 16.5;
    private static final double wheelBase = 16.5;

    private final AnalogInput frontLeftRotationEncoder = new AnalogInput(AnalogDevices.frontLeftRotationEncoderPort);
    private final AnalogInput frontRightRotationEncoder = new AnalogInput(AnalogDevices.frontRightRotationEncoderPort);
    private final AnalogInput rearLeftRotationEncoder = new AnalogInput(AnalogDevices.rearLeftRotationEncoderPort);
    private final AnalogInput rearRightRotationEncoder = new AnalogInput(AnalogDevices.rearRightRotationEncoderPort);

    private final SwerveModule frontLeft = 
        new Mk2SwerveModuleBuilder(
            new Vector2(trackWidth / 2.0, wheelBase / 2.0))
            .angleEncoder(
                    frontLeftRotationEncoder, 0.0)
            .angleMotor(
                    new CANSparkMax(CANDevices.frontLeftRotationMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                    new CANSparkMax(CANDevices.frontLeftDriveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    private final SwerveModule frontRight = 
        new Mk2SwerveModuleBuilder(
            new Vector2(trackWidth / 2.0, -wheelBase / 2.0))
            .angleEncoder(
                    frontRightRotationEncoder, 0.0)
            .angleMotor(
                    new CANSparkMax(CANDevices.frontRightRotationMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                    new CANSparkMax(CANDevices.frontRightDriveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    private final SwerveModule rearLeft = 
        new Mk2SwerveModuleBuilder(
            new Vector2(-trackWidth / 2.0, wheelBase / 2.0))
            .angleEncoder(
                    rearLeftRotationEncoder, 0.0)
            .angleMotor(
                    new CANSparkMax(CANDevices.rearLeftRotationMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                    new CANSparkMax(CANDevices.rearLeftDriveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    private final SwerveModule rearRight = 
        new Mk2SwerveModuleBuilder(
            new Vector2(-trackWidth / 2.0, -wheelBase / 2.0))
            .angleEncoder(
                    rearRightRotationEncoder, 0.0)
            .angleMotor(
                    new CANSparkMax(CANDevices.rearRightRotationMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                    new CANSparkMax(CANDevices.rearRightDriveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    private final SwerveDriveKinematics kinematics = 
        new SwerveDriveKinematics(
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0));

    public DriveSubsystem() {

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

    }

    public void drive(Translation2d translation, double rotation) {

        rotation *= 2.0 / Math.hypot(wheelBase, trackWidth);

        ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        
        frontLeft.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRight.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        rearLeft.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        rearRight.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());

    }

    public void resetEncoders() {

        frontLeftRotationEncoder.resetAccumulator();
        frontRightRotationEncoder.resetAccumulator();
        rearLeftRotationEncoder.resetAccumulator();
        rearRightRotationEncoder.resetAccumulator();

    }

}
