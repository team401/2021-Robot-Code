package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PneumaticChannels;

public class BallSubsystem extends SubsystemBase {
    
    private final SpeedControllerGroup flywheel = 
        new SpeedControllerGroup(
            new WPI_TalonFX(CANDevices.leftFlywheelMotorId), 
            new WPI_TalonFX(CANDevices.rightFlywheelMotorId));

    private final CANSparkMax kickerMotor = new CANSparkMax(CANDevices.kickerMotorId, MotorType.kBrushless);
    private final CANSparkMax intakeMotor = new CANSparkMax(CANDevices.intakeMotorId, MotorType.kBrushless);

    private final DoubleSolenoid leftIntakeSolenoid = 
        new DoubleSolenoid(
            PneumaticChannels.leftIntakeSolenoidChannels[0], 
            PneumaticChannels.leftIntakeSolenoidChannels[1]);

    private final DoubleSolenoid rightIntakeSolenoid = 
        new DoubleSolenoid(
            PneumaticChannels.rightIntakeSolenoidChannels[0], 
            PneumaticChannels.rightIntakeSolenoidChannels[1]);

    public BallSubsystem() {}

}
