package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

    /**
     * Class to represent and handle a swerve module
     * A module's state is measured by a CANCoder for the absolute position, integrated CANEncoder for relative position
     * for both rotation and linear movement
     */

    private static final double rotationkP = 1;
    private static final double rotationkD = 0.5;

    private static final double drivekP = 0.01;

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final CANEncoder driveEncoder;
    private final CANEncoder rotationEncoder;

    private final CANCoder canCoder;

    //absolute offset for the CANCoder so that the wheels can be aligned when the robot is turned on
    private final Rotation2d offset;

    private final CANPIDController rotationController;
    private final CANPIDController driveController;

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
        driveController = driveMotor.getPIDController();

        rotationController.setP(rotationkP);
        rotationController.setD(rotationkD);

        driveController.setP(drivekP);

        //set the output of the drive encoder to be in radians for linear measurement
        driveEncoder.setPositionConversionFactor(
            2.0 * Math.PI / DriveConstants.driveWheelGearReduction
        );

        //set the output of the drive encoder to be in radians per second for velocity measurement
        driveEncoder.setVelocityConversionFactor(
            2.0 * Math.PI / 60 / DriveConstants.driveWheelGearReduction
        );

        //set the output of the rotation encoder to be in radians
        rotationEncoder.setPositionConversionFactor(2 * Math.PI / DriveConstants.rotationWheelGearReduction);

        //configure the CANCoder to output in unsigned (wrap around from 360 to 0 degrees)
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    }

    public void resetDistance() {

        driveEncoder.setPosition(0.0);

    }

    public double getDriveDistanceRadians() {

        return driveEncoder.getPosition();

    }
    
    public Rotation2d getCanCoderAngle() {

        double unsignedAngle = (Units.degreesToRadians(canCoder.getAbsolutePosition()) - offset.getRadians()) % (2 * Math.PI);

        return new Rotation2d(unsignedAngle);

    }

    public Rotation2d getCanEncoderAngle() {

        double unsignedAngle = rotationEncoder.getPosition() % (2 * Math.PI);

        if (unsignedAngle < 0) unsignedAngle += 2 * Math.PI;

        return new Rotation2d(unsignedAngle);

    }

    public double getCurrentVelocityRadiansPerSecond() {

        return driveEncoder.getVelocity();
        
    }

    public double getCurrentVelocityMetersPerSecond() {

        return driveEncoder.getVelocity() * (DriveConstants.wheelDiameterMeters / 2.0);

    }

    //calculate the angle motor setpoint based on the desired angle and the current angle measurement
    public double calculateAdjustedAngle(double targetAngle, double currentAngle) {

        double modAngle = currentAngle % (2.0 * Math.PI);

        if (modAngle < 0.0) modAngle += 2.0 * Math.PI;
        
        double newTarget = targetAngle + currentAngle - modAngle;

        if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
        else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;

        return newTarget;

    }

    //initialize the integrated CANCoder to offset measurement by the CANCoder reading
    public void initRotationOffset() {

        rotationEncoder.setPosition(getCanCoderAngle().getRadians());

    }

    /**
     * Method to set the desired state of the swerve module
     * Parameter: 
     * SwerveModuleState object that holds a desired linear and rotational setpoint
     * Uses PID and a feedforward to control the output
     */
    public void setDesiredStateClosedLoop(SwerveModuleState desiredState) {
        
        SwerveModuleState state = desiredState;
        rotationController.setReference(
            calculateAdjustedAngle(
                state.angle.getRadians(),
                rotationEncoder.getPosition()),
            ControlType.kPosition
        );

        double speedRadPerSec = desiredState.speedMetersPerSecond / (DriveConstants.wheelDiameterMeters / 2);

        driveController.setReference(
            speedRadPerSec, 
            ControlType.kVelocity, 
            0, 
            DriveConstants.driveFF.calculate(speedRadPerSec)
        );

    }

    public void resetEncoders() {

        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(0);

    }
    
}