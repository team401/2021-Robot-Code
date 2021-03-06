package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {

    private static final double drivekP = 1.5;
    private static final double drivekI = 0.0;
    private static final double drivekD = 0.5;

    private static final double rotationkP = 1;
    private static final double rotationkI = 0;
    private static final double rotationkD = 0.5;

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final CANEncoder driveEncoder;
    private final CANEncoder rotationEncoder;

    private final AnalogInput analogEncoder;

    private final Rotation2d offset;

    private final CANPIDController driveController;
    private final CANPIDController rotationController;

    public SwerveModule(
        int driveMotorId, 
        int rotationMotorId,
        int analogEncoderPort, 
        double measuredOffsetRadians
        ) {

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        analogEncoder = new AnalogInput(analogEncoderPort);

        offset = new Rotation2d(measuredOffsetRadians);

        driveMotor.setIdleMode(IdleMode.kBrake);

        driveController = driveMotor.getPIDController();
        rotationController = rotationMotor.getPIDController();

        driveController.setP(drivekP);
        driveController.setI(drivekI);
        driveController.setD(drivekD);

        rotationController.setP(rotationkP);
        rotationController.setI(rotationkI);
        rotationController.setD(rotationkD);
        //rotationController.setSmartMotionMaxVelocity(DriveConstants.rotationMotorMaxSpeedRadPerSec, 0);
        //rotationController.setSmartMotionMaxAccel(DriveConstants.rotationMotorMaxAccelRadPerSecSq, 0);

        driveEncoder.setPositionConversionFactor(
            DriveConstants.wheelDiameterMeters * Math.PI / DriveConstants.driveWheelGearReduction);

        driveEncoder.setVelocityConversionFactor(
            DriveConstants.wheelDiameterMeters * Math.PI / 60 / DriveConstants.driveWheelGearReduction);

        rotationEncoder.setPositionConversionFactor(
            2 * Math.PI / DriveConstants.rotationWheelGearReduction
        );

        rotationEncoder.setVelocityConversionFactor(Math.PI / (60 / 2) / DriveConstants.rotationWheelGearReduction);

    }

    public Rotation2d getAnalogEncoderAngle() {

            double angle = 
                (1.0 - analogEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI;

            angle += offset.getRadians();
            angle %= 2.0 * Math.PI;

            if (angle < 0.0) angle += 2.0 * Math.PI;

        return new Rotation2d(angle);

    }

    public Rotation2d getCurrentAngle() {

        double currentAngle = rotationEncoder.getPosition();
        double unsignedAngle = currentAngle % (2 * Math.PI);

        return new Rotation2d(unsignedAngle);

    }

    public double getCurrentVelocity() {

        return driveEncoder.getVelocity();

    }

    public void initRotationMotorOffset() {

        rotationEncoder.setPosition(getAnalogEncoderAngle().getRadians());

    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getCurrentAngle());

        double targetAngle = state.angle.getRadians();
        double currentAngle = rotationEncoder.getPosition();
        double modAngle = currentAngle % (2.0 * Math.PI);

        if (modAngle < 0.0) modAngle += 2.0 * Math.PI;
        
        double newTarget = targetAngle + currentAngle - modAngle;

        if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
        else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;

        rotationController.setReference(newTarget, ControlType.kPosition);
        driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

    }

    public void resetEncoders() {

        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(0);

    }
    
}