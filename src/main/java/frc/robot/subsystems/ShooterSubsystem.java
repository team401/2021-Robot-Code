package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PneumaticChannels;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonFX leftFLywheelMotor = new WPI_TalonFX(CANDevices.leftFlywheelMotorId);
    private final WPI_TalonFX rightFlywheelMotor = new WPI_TalonFX(CANDevices.rightFlywheelMotorId);
    
    private final SpeedControllerGroup flywheel = new SpeedControllerGroup(leftFLywheelMotor, rightFlywheelMotor);

    private final DoubleSolenoid hoodSolenoid = 
        new DoubleSolenoid(
            PneumaticChannels.hoodSolenoidChannels[0], 
            PneumaticChannels.hoodSolenoidChannels[1]);

    public ShooterSubsystem() {

        leftFLywheelMotor.setInverted(true);

    }

}
