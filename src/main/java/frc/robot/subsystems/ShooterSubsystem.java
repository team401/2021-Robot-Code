package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SuperstructureConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonFX leftFlywheelMotor = new WPI_TalonFX(CANDevices.leftFlywheelMotorId);
    private final WPI_TalonFX rightFlywheelMotor = new WPI_TalonFX(CANDevices.rightFlywheelMotorId);
    
    private final CANSparkMax kickerMotor = new CANSparkMax(CANDevices.kickerMotorId, MotorType.kBrushless);

    private final SpeedControllerGroup flywheel = new SpeedControllerGroup(leftFlywheelMotor, rightFlywheelMotor);

    public ShooterSubsystem() {

        rightFlywheelMotor.setInverted(true);

    }

    public void runShooterPercent(double speed) {

        flywheel.set(speed);

    }

    public void stopShooter() {

        flywheel.set(0);

    }

    public void stopKicker() {

        kickerMotor.set(0);

    }

    public double getFlywheelVelRadPerSec() {

        return (((leftFlywheelMotor.getSelectedSensorVelocity() + rightFlywheelMotor.getSelectedSensorVelocity()) / 2) 
        / 2048 * (2 * Math.PI) * 10) * SuperstructureConstants.flywheelGearRatio; // raw sensor unit/100ms * 1 rotation/2048 units Talon * 2pi rad/1 rotation / 10 100ms/sec = rad/sec

    }

    public void runKicker() {

        

        kickerMotor.set(SuperstructureConstants.kickerPower);

    }

    public void reverseKicker() {

        kickerMotor.set(-SuperstructureConstants.kickerPower);

    }

}
