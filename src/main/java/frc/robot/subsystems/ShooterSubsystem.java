package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PneumaticChannels;
import frc.robot.Constants.SuperstructureConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonFX leftFlywheelMotor = new WPI_TalonFX(CANDevices.leftFlywheelMotorId);
    private final WPI_TalonFX rightFlywheelMotor = new WPI_TalonFX(CANDevices.rightFlywheelMotorId);
    
    private final CANSparkMax kickerMotor = new CANSparkMax(CANDevices.kickerMotorId, MotorType.kBrushless);

    private final SpeedControllerGroup flywheel = new SpeedControllerGroup(leftFlywheelMotor, rightFlywheelMotor);

    private final PIDController flywheelController = new PIDController(0.0, 0.0, 0.0);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

    private final DoubleSolenoid hoodSolenoid = 
        new DoubleSolenoid(
            PneumaticChannels.hoodSolenoidChannels[0], 
            PneumaticChannels.hoodSolenoidChannels[1]
        );

    public ShooterSubsystem() {

        leftFlywheelMotor.setInverted(true);

    }

    public void setHoodState(boolean state) {

        if (state) hoodSolenoid.set(Value.kForward);
        else hoodSolenoid.set(Value.kReverse);

    }

    public void runFlywheel(double desiredVelocityRadPerSec) {

        double power = flywheelController.calculate(getFlywheelVelRadPerSec(), desiredVelocityRadPerSec);

        double calculatedFeedForward = feedforward.calculate(flywheelController.getSetpoint());

        flywheel.setVoltage(power + calculatedFeedForward);

    }

    public double getFlywheelVelRadPerSec() {

        return ((leftFlywheelMotor.getSelectedSensorVelocity() + rightFlywheelMotor.getSelectedSensorVelocity()) / 2) * 600
            / 2048 * (2 * Math.PI) / SuperstructureConstants.flywheelGearRatio; 

    }

    public Boolean isFlywheelAtSetpoint(double desiredVelocityRadPerSec) {

        return flywheelController.atSetpoint();

    }

    public void runKicker() {

        kickerMotor.set(SuperstructureConstants.kickerPower);

    }

}
