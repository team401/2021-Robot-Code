package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

    private static final double rotationkP = 1;
    private static final double rotationkI = 0;
    private static final double rotationkD = 0.5;

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final CANEncoder driveEncoder;
    private final CANEncoder rotationEncoder;

    private final CANCoder canCoder;

    private final Rotation2d offset;

    private final CANPIDController rotationController;

    public SwerveModule(
        int driveMotorId, 
        int rotationMotorId,
        int canCoderId,
        double measuredOffsetRadians
    ) {

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        canCoder = new CANCoder(canCoderId);

        offset = new Rotation2d(measuredOffsetRadians);

        driveMotor.setIdleMode(IdleMode.kBrake);
        rotationMotor.setIdleMode(IdleMode.kCoast);

        rotationController = rotationMotor.getPIDController();

        rotationController.setP(rotationkP);
        rotationController.setI(rotationkI);
        rotationController.setD(rotationkD);

        driveEncoder.setPositionConversionFactor(
                DriveConstants.wheelDiameterMeters * Math.PI / DriveConstants.driveWheelGearReduction);

        driveEncoder.setVelocityConversionFactor(
                DriveConstants.wheelDiameterMeters * Math.PI / 60 / DriveConstants.driveWheelGearReduction);

        rotationEncoder.setPositionConversionFactor(2 * Math.PI / DriveConstants.rotationWheelGearReduction);

        rotationEncoder.setVelocityConversionFactor(Math.PI / (60 / 2) / DriveConstants.rotationWheelGearReduction);

        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    }

    public Rotation2d getCanCoderAngle() {

        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition()).plus(offset);

    }

    public Rotation2d getCanEncoderAngle() {

        double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);

        return new Rotation2d(unsignedAngle);

    }

    public double getCurrentVelocity() {

        return driveEncoder.getVelocity();

    }

    public double calculateAdjustedAngle(double targetAngle, double currentAngle) {

        double modAngle = currentAngle % (2.0 * Math.PI);

        if (modAngle < 0.0) modAngle += 2.0 * Math.PI;
        
        double newTarget = targetAngle + currentAngle - modAngle;

        if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
        else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;

        return newTarget;

    }

    public void initRotationMotorOffset() {

        rotationEncoder.setPosition(getCanCoderAngle().getRadians());

    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getCanEncoderAngle());

        rotationController.setReference(
            calculateAdjustedAngle(
                state.angle.getRadians(), 
                rotationEncoder.getPosition()), 
            ControlType.kPosition
        );

        driveMotor.set(state.speedMetersPerSecond);

    }

    public void resetEncoders() {

        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(0);

    }
    
}