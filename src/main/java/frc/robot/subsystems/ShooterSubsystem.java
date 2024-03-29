package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SuperstructureConstants;

public class ShooterSubsystem extends SubsystemBase {

    /**
     * Subsystem that controls the shooter of the robot
     */

    private final WPI_TalonFX leftFlywheelMotor = new WPI_TalonFX(CANDevices.leftFlywheelMotorId);
    private final WPI_TalonFX rightFlywheelMotor = new WPI_TalonFX(CANDevices.rightFlywheelMotorId);
    
    private final CANSparkMax kickerMotor = new CANSparkMax(CANDevices.kickerMotorId, MotorType.kBrushless);

    private final SpeedControllerGroup flywheel = new SpeedControllerGroup(leftFlywheelMotor, rightFlywheelMotor);

    private final ProfiledPIDController controller = new ProfiledPIDController(
        0.005, 0.005, 0,
        new TrapezoidProfile.Constraints(
            Units.rotationsPerMinuteToRadiansPerSecond(6380) * SuperstructureConstants.flywheelGearRatio,
            Units.rotationsPerMinuteToRadiansPerSecond(2000)
        )
    );

    private Timer timer = new Timer();

    private double desiredSpeed = 0;

    public ShooterSubsystem() {

        rightFlywheelMotor.setInverted(true);

        controller.setTolerance(Units.rotationsPerMinuteToRadiansPerSecond(20));

        timer.start();
        timer.reset();

    }

    @Override
    public void periodic() {

        if (!(Math.abs(desiredSpeed - getFlywheelVelRadPerSec()) < 50)) timer.reset();

    }

    public double getFlywheelVelRadPerSec() {

        return (((leftFlywheelMotor.getSelectedSensorVelocity()
        / 2048 * (2 * Math.PI) * 10) * SuperstructureConstants.flywheelGearRatio)); 
        // raw sensor unit/100ms * 1 rotation/2048 units Talon * 2pi rad/1 rotation / 10 100ms/sec = rad/sec

    }

    public void runVelocityProfileController(double desiredSpeedRadPerSec) {

        desiredSpeed = desiredSpeedRadPerSec;

        double powerOut = controller.calculate(
                getFlywheelVelRadPerSec(), 
                desiredSpeedRadPerSec
            );

        flywheel.set(powerOut);

        SmartDashboard.putNumber("current", Units.radiansPerSecondToRotationsPerMinute(getFlywheelVelRadPerSec()));

    }

    public void runShooterPercent(double speed) {

        flywheel.set(speed);

    }

    public void stopShooter() {

        flywheel.set(0);

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

    public boolean atGoal() {

        return timer.get() >= 0.25;

    }

}
