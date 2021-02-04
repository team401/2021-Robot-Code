package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants.AnalogDevices;
import frc.robot.Constants.CANDevices;
import frc.robot.commands.OperatorControl;

import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;

public class DriveSubsystem extends Subsystem {
    private static final double trackWidth = 16.5;
    private static final double wheelBase = 16.5;

    private static DriveSubsystem instance;

    private final SwerveModule frontLeftModule = 
        new Mk2SwerveModuleBuilder(
            new Vector2(trackWidth / 2.0, wheelBase / 2.0))
            .angleEncoder(
                    new AnalogInput(AnalogDevices.frontLeftRotationEncoderPort), 0.0)
            .angleMotor(
                    new CANSparkMax(CANDevices.frontLeftRotationMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                    new CANSparkMax(CANDevices.frontLeftDriveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    private final SwerveModule frontRightModule = 
        new Mk2SwerveModuleBuilder(
            new Vector2(trackWidth / 2.0, -wheelBase / 2.0))
            .angleEncoder(
                    new AnalogInput(AnalogDevices.frontRightRotationEncoderPort), 0.0)
            .angleMotor(
                    new CANSparkMax(CANDevices.frontRightRotationMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                    new CANSparkMax(CANDevices.frontRightDriveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    private final SwerveModule backLeftModule = 
        new Mk2SwerveModuleBuilder(
            new Vector2(-trackWidth / 2.0, wheelBase / 2.0))
            .angleEncoder(
                    new AnalogInput(AnalogDevices.rearLeftRotationEncoderPort), 0.0)
            .angleMotor(
                    new CANSparkMax(CANDevices.rearLeftRotationMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                    new CANSparkMax(CANDevices.rearLeftDriveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    private final SwerveModule backRightModule = 
        new Mk2SwerveModuleBuilder(
            new Vector2(-trackWidth / 2.0, -wheelBase / 2.0))
            .angleEncoder(
                    new AnalogInput(AnalogDevices.rearRightRotationEncoderPort), 0.0)
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

    public DriveSubsystem() { }

    public static DriveSubsystem getInstance() {

        if (instance == null) instance = new DriveSubsystem();

        return instance;

    }

    @Override
    public void periodic() {

        frontLeftModule.updateSensors();
        frontRightModule.updateSensors();
        backLeftModule.updateSensors();
        backRightModule.updateSensors();

        frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
        frontRightModule.updateState(TimedRobot.kDefaultPeriod);
        backLeftModule.updateState(TimedRobot.kDefaultPeriod);
        backRightModule.updateState(TimedRobot.kDefaultPeriod);

    }

    public void drive(Translation2d translation, double rotation) {

        rotation *= 2.0 / Math.hypot(wheelBase, trackWidth);

        ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        
        frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());

    }


    @Override
    protected void initDefaultCommand() {

        setDefaultCommand(new OperatorControl());

    }
}
