package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveModule extends SubsystemBase {

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final CANCoder canCoder;

    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController rotationController;
    
    private final Rotation2d offset;

    public DriveModule(int driveID, int rotationID, int encoderID, double rotationOffset) {

        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        canCoder = new CANCoder(encoderID);

        offset = new Rotation2d(rotationOffset);

        driveMotor.setIdleMode(IdleMode.kBrake);
        rotationMotor.setIdleMode(IdleMode.kCoast);

        driveController = driveMotor.getPIDController();
        rotationController = rotationMotor.getPIDController();
        
        driveController.setP(DriveConstants.driveKp);
        driveController.setD(DriveConstants.driveKd);

        rotationController.setP(DriveConstants.rotationKp);
        rotationController.setD(DriveConstants.rotationKd);

        driveEncoder.setPositionConversionFactor(2.0 * Math.PI / DriveConstants.driveWheelGearReduction);
        driveEncoder.setVelocityConversionFactor(2.0 * Math.PI / 60 / DriveConstants.driveWheelGearReduction);
        rotationEncoder.setPositionConversionFactor(2 * Math.PI / DriveConstants.rotationWheelGearReduction);

        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState state = desiredState;
        rotationController.setReference(
            calculateAdjustedAngle(
                state.angle.getRadians(),
                rotationEncoder.getPosition()),
            ControlType.kPosition
        );

        double speedRadPerSec = desiredState.speedMetersPerSecond / DriveConstants.wheelRadiusM;

        driveController.setReference(
            speedRadPerSec, 
            ControlType.kVelocity, 
            0, 
            DriveConstants.driveFF.calculate(speedRadPerSec)
        );

    }

    public double getDrivePosition() {  
        return driveEncoder.getPosition();
    }

    public double getRotationPosition() {
        double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);

        if (unsignedAngle < 0) unsignedAngle += 2 * Math.PI;

        return unsignedAngle;
    }

    public double getCanCoderAngle() {
        return (Units.degreesToRadians(canCoder.getAbsolutePosition()) - offset.getRadians()) % (2 * Math.PI);
    }

    public void zeroEncoders() {
        rotationEncoder.setPosition(getCanCoderAngle());
        driveEncoder.setPosition(0.0);
    }

    public double getDriveVelocityMPerS() {
        return driveEncoder.getVelocity() * DriveConstants.wheelRadiusM;
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocityMPerS(), new Rotation2d(getRotationPosition()));
    }

    public double calculateAdjustedAngle(double targetAngle, double currentAngle) {

        double modAngle = currentAngle % (2.0 * Math.PI);

        if (modAngle < 0.0) modAngle += 2.0 * Math.PI;
        
        double newTarget = targetAngle + currentAngle - modAngle;

        if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
        else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;

        return newTarget;

    }
    
}