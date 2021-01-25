package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;
    
    private final CANEncoder driveEncoder;
    private final AnalogEncoder rotationEncoder;

    private final PIDController driveController = 
        new PIDController(
            ModuleConstants.driveControllerKp, 
            ModuleConstants.driveControllerKi, 
            ModuleConstants.driveControllerKd);

    private final ProfiledPIDController rotationController = 
        new ProfiledPIDController(
            ModuleConstants.rotationControllerKp,
            ModuleConstants.rotationControllerKi,
            ModuleConstants.rotationControllerKd,
            new TrapezoidProfile.Constraints(
                ModuleConstants.maxAngularVel,
                ModuleConstants.maxAngularAccel));

    private final SimpleMotorFeedforward driveFeedforward = 
        new SimpleMotorFeedforward(
            ModuleConstants.driveKs, 
            ModuleConstants.driveKv);

    private final SimpleMotorFeedforward rotationFeedforward =
        new SimpleMotorFeedforward(
            ModuleConstants.rotationKs, 
            ModuleConstants.rotationKv);

    public SwerveModule(int driveMotorId, int rotationMotorId, int rotationEncoderPort) {

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = new AnalogEncoder(new AnalogInput(rotationEncoderPort));
        
        rotationEncoder.setDistancePerRotation(360 / ModuleConstants.rotationGearingRatio);

        driveEncoder.setPositionConversionFactor(Math.PI * ModuleConstants.wheelDiameterMeters * ModuleConstants.driveGearingRatio);
        driveEncoder.setVelocityConversionFactor(
            Math.PI * ModuleConstants.wheelDiameterMeters * ModuleConstants.driveGearingRatio / 60.0);

        rotationController.enableContinuousInput(-180, 180);

    }

    public SwerveModuleState getState() {

        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(rotationEncoder.getDistance()));

    }

    public void setDesiredState(SwerveModuleState state) {

        final var driveOut = driveController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);
        final var rotationOut = rotationController.calculate(rotationEncoder.getDistance(), state.angle.getDegrees());

        driveMotor.set(driveOut);
        rotationMotor.set(rotationOut);

    }

    public void resetEncoders() {

        driveEncoder.setPosition(0);
        rotationEncoder.reset();

    }

}
