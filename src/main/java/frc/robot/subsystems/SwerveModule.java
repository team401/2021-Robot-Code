package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    
    private static final double drivekP = 0.0;
    private static final double drivekI = 0.0;
    private static final double drivekD = 0.0;

    private static final double rotationkP = 0.1;
    private static final double rotationkI = 0.0;
    private static final double rotationkD = 0.0;

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final CANEncoder driveEncoder;
    private final CANEncoder rotationEncoder;

    private final CANCoder externalRotationEncoder;

    private final Rotation2d offset;

    private final CANPIDController driveController;
    private final ProfiledPIDController rotationController;

    public SwerveModule(int driveMotorId, int rotationMotorId, int externalRotationEncoderId, double measuredOffsetRadians) {

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        externalRotationEncoder = new CANCoder(externalRotationEncoderId);

        offset = new Rotation2d(measuredOffsetRadians);

        driveMotor.setIdleMode(IdleMode.kBrake);

        externalRotationEncoder.setPositionToAbsolute();
        
        driveController = driveMotor.getPIDController();

        rotationController = 
            new ProfiledPIDController(
                rotationkP, 
                rotationkI, 
                rotationkD, 
                new TrapezoidProfile.Constraints(
                    DriveConstants.rotationMotorMaxSpeedRadPerSec,
                    DriveConstants.rotationMotorMaxAccelRadPerSecSq
                )
            );

        driveController.setP(drivekP);
        driveController.setI(drivekI);
        driveController.setD(drivekD);
        
        driveEncoder.setPositionConversionFactor(
            DriveConstants.wheelDiameterMeters * Math.PI / DriveConstants.driveWheelGearReduction 
        );

        driveEncoder.setVelocityConversionFactor(
            DriveConstants.wheelDiameterMeters * Math.PI / 60 / DriveConstants.driveWheelGearReduction 
        );

        rotationEncoder.setPositionConversionFactor(
            2 * Math.PI / DriveConstants.rotationWheelGearReduction
        );

        rotationEncoder.setVelocityConversionFactor(
            Math.PI / (60 / 2) / DriveConstants.rotationWheelGearReduction
        );

    }

    public Rotation2d getCurrentAngle() {

        return new Rotation2d(externalRotationEncoder.getPosition() + offset.getRadians());

    }

    public double getCurrentVelocity() {

        return driveEncoder.getVelocity();

    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getCurrentAngle());

        driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        double rotationOut = rotationController.calculate(getCurrentAngle().getRadians(), state.angle.getRadians());
        rotationMotor.setVoltage(rotationOut);

    }

    public void resetEncoders() {

        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(0);

    }
    
}