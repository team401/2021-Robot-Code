package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SuperstructureConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonFX leftFlywheelMotor = new WPI_TalonFX(CANDevices.leftFlywheelMotorId);
    private final WPI_TalonFX rightFlywheelMotor = new WPI_TalonFX(CANDevices.rightFlywheelMotorId);
    
    private final CANSparkMax kickerMotor = new CANSparkMax(CANDevices.kickerMotorId, MotorType.kBrushless);

    private final SpeedControllerGroup flywheel = new SpeedControllerGroup(leftFlywheelMotor, rightFlywheelMotor);

    private final ProfiledPIDController flywheelController = 
        new ProfiledPIDController(
            0.2, 
            0.0, 
            0.0,
            new TrapezoidProfile.Constraints(
                100,
                1
            )
        );

    public ShooterSubsystem() {

        rightFlywheelMotor.setInverted(true);

    }

    public void runShooterPercent() {

        flywheel.set(SuperstructureConstants.shooterPower);

    }

    public void stopShooter() {

        flywheel.set(0);

    }

    public void runFlywheel(double desiredVelocityRadPerSec) {

        double power = flywheelController.calculate(getFlywheelVelRadPerSec(), desiredVelocityRadPerSec);

        flywheel.set(power);

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

    public void stopKicker() {

        kickerMotor.set(0);
    
    }

    public void reverseKicker() {

        kickerMotor.set(-SuperstructureConstants.kickerPower);

    }

}
