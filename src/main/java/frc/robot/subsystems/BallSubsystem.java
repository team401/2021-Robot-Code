package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOChannels;
import frc.robot.Constants.PneumaticChannels;
import frc.robot.Constants.SuperstructureConstants;

public class BallSubsystem extends SubsystemBase {

    private final WPI_TalonFX leftFlywheelMotor = new WPI_TalonFX(CANDevices.leftFlywheelMotorId);
    private final WPI_TalonFX rightFlywheelMotor = new WPI_TalonFX(CANDevices.rightFlywheelMotorId);
    
    private final SpeedControllerGroup flywheel = 
        new SpeedControllerGroup(
            leftFlywheelMotor,
            rightFlywheelMotor);

    private final CANSparkMax kickerMotor = new CANSparkMax(CANDevices.kickerMotorId, MotorType.kBrushless);
    private final CANSparkMax intakeMotor = new CANSparkMax(CANDevices.intakeMotorId, MotorType.kBrushless);
    private final CANSparkMax conveyorMotor = new CANSparkMax(CANDevices.conveyorMotorId, MotorType.kBrushless);

    private final DoubleSolenoid leftIntakeSolenoid = 
        new DoubleSolenoid(
            PneumaticChannels.leftIntakeSolenoidChannels[0], 
            PneumaticChannels.leftIntakeSolenoidChannels[1]);

    private final DoubleSolenoid rightIntakeSolenoid = 
        new DoubleSolenoid(
            PneumaticChannels.rightIntakeSolenoidChannels[0], 
            PneumaticChannels.rightIntakeSolenoidChannels[1]);

    private final DoubleSolenoid hoodSolenoid = 
        new DoubleSolenoid(
            PneumaticChannels.hoodSolenoidChannels[0], 
            PneumaticChannels.hoodSolenoidChannels[1]);

    private final DigitalInput bottomBanner = new DigitalInput(DIOChannels.topBannerPort);
    private final DigitalInput topBanner = new DigitalInput(DIOChannels.bottomBannerPort);

    private final PIDController flywheelController = 
        new PIDController(
            0.0, 
            0.0,
            0.0);

    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.0, 0.0);

    public BallSubsystem() {

        intakeMotor.setIdleMode(IdleMode.kBrake);
        kickerMotor.setIdleMode(IdleMode.kBrake);

        leftFlywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightFlywheelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    }

    public void runIntake() {

        intakeMotor.set(SuperstructureConstants.intakingPower);

    }

    public void runConveyor() {

        conveyorMotor.set(SuperstructureConstants.conveyorPower);

    }

    public void runKicker() {

        kickerMotor.set(SuperstructureConstants.kickerPower);

    }

    public void deployIntake() {

        leftIntakeSolenoid.set(Value.kForward);
        rightIntakeSolenoid.set(Value.kForward);

    }

    public void retractIntake() {

        leftIntakeSolenoid.set(Value.kReverse);
        rightIntakeSolenoid.set(Value.kReverse);

    }

    public void setHoodState(boolean state) {

        if (state) hoodSolenoid.set(Value.kForward);
        else hoodSolenoid.set(Value.kReverse);

    }

    public Boolean getTopBannerState() {

        return topBanner.get();

    }

    public Boolean getBottomBannerState() {

        return bottomBanner.get();

    }

    public double getFlywheelVelRadPerSec() {

        return ((leftFlywheelMotor.getSelectedSensorVelocity() * 600) 
        * (2048 / SuperstructureConstants.flywheelGearRatio)) / (2 * Math.PI); // Double check units

    }

    public void runFlywheel(double desiredVelocityRadPerSec) {

        double power = flywheelController.calculate(getFlywheelVelRadPerSec(), desiredVelocityRadPerSec);

        double calculatedFeedForward = feedForward.calculate(flywheelController.getSetpoint());

        flywheel.setVoltage(power + calculatedFeedForward);

    }

    public Boolean isFlywheelAtSetpoint(double desiredVelocityRadPerSec) {

        return flywheelController.atSetpoint();

    }

}  