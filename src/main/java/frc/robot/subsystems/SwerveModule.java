package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    
    private static final double drivekP = 1.5;
    private static final double drivekI = 0.0;
    private static final double drivekD = 0.0001;
    private static final double driveFF = 0.0;

    private static final double rotationkP = 1.5;
    private static final double rotationkI = 0.0;
    private static final double rotationkD = 0.0;
    private static final double rotationFF = 0.0;

    private static final double rotationControllerMaxSpeedRadPerSec = .1;
    private static final double rotationControllerMaxAccelRadPerSecSq = .1;

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final CANEncoder driveEncoder;
    private final AnalogEncoder rotationEncoder;

    private final Rotation2d offset;

    private final CANPIDController driveController;
    private final CANPIDController rotationController;


    public SwerveModule(int driveMotorId, int rotationMotorId, int rotationEncoderPort, double measuredOffsetRadians) {

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = new AnalogEncoder(new AnalogInput(rotationEncoderPort));

        driveController = driveMotor.getPIDController();
        rotationController = rotationMotor.getPIDController();

        driveController.setP(drivekP);
        driveController.setI(drivekI);
        driveController.setD(drivekD);
        driveController.setFF(driveFF);

        rotationController.setP(rotationkP);
        rotationController.setI(rotationkI);
        rotationController.setD(rotationkD);
        rotationController.setFF(rotationFF);
        rotationController.setSmartMotionMaxVelocity(rotationControllerMaxSpeedRadPerSec, 0);
        rotationController.setSmartMotionMaxAccel(rotationControllerMaxAccelRadPerSecSq, 0);

        driveEncoder.setPositionConversionFactor(
            DriveConstants.wheelDiameterMeters * Math.PI / DriveConstants.driveWheelGearReduction 
        );

        driveEncoder.setVelocityConversionFactor(
            DriveConstants.wheelDiameterMeters * Math.PI / 60 / DriveConstants.driveWheelGearReduction 
        );

        offset = new Rotation2d(measuredOffsetRadians);

        driveMotor.setIdleMode(IdleMode.kBrake);

    }

    public Rotation2d getCurrentAngle() {

        return new Rotation2d(rotationEncoder.getDistance() % (2 * Math.PI) + offset.getRadians());

    }

    public double getCurrentVelocity() {

        return driveEncoder.getVelocity();

    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getCurrentAngle());

        driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        rotationController.setReference(state.angle.getRadians(), ControlType.kSmartMotion);

    }

    public void resetEncoders() {

        driveEncoder.setPosition(0);
        rotationEncoder.reset();

    }
    
}
