package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SuperstructureConstants;

public class Shooter extends SubsystemBase {

    private final WPI_VictorSPX flywheelMotor = new WPI_VictorSPX(CANDevices.flywheelMotorId);

    //Shoves ball into flywheel from indexing chamber, lauching it
    private final CANSparkMax kickerMotor = new CANSparkMax(CANDevices.kickerMotorId, MotorType.kBrushless);

    //None of the uses for this subsystem really *need* PID, but this a good case for practising using it 
    private final ProfiledPIDController controller = new ProfiledPIDController(
        0.005, 0.005, 0,
        new TrapezoidProfile.Constraints(
            Units.rotationsPerMinuteToRadiansPerSecond(6380) * SuperstructureConstants.flywheelGearRatio,
            Units.rotationsPerMinuteToRadiansPerSecond(2000)
        )
    );

    private Timer timer = new Timer();

    private double powerReduction = 0;
    private double desiredSpeed = 0;

    public Shooter() {

        flywheelMotor.setInverted(true);

        controller.setTolerance(Units.rotationsPerMinuteToRadiansPerSecond(20));

        timer.start();
        timer.reset();

        SmartDashboard.putNumber("Flywheel Power Reduction", 0.5);

    }

    @Override
    public void periodic() {
        powerReduction = SmartDashboard.getNumber("Flywheel Power Reduction", 0.5);

        if (!(Math.abs(desiredSpeed - getFlywheelVelRadPerSec()) < 50)) timer.reset();

    }

    public double getFlywheelVelRadPerSec() {

        return (((flywheelMotor.getSelectedSensorVelocity()
        / 2048 * (2 * Math.PI) * 10) * SuperstructureConstants.flywheelGearRatio));
        //Unit Conversions: raw sensor unit/100ms * 1 rotation/2048 Talon units  * 2pi rad/1 rotation / 10 100ms/sec = rad/sec

    }

    public void runVelocityProfileController(double desiredSpeedRadPerSec) {

        desiredSpeed = desiredSpeedRadPerSec * powerReduction;

        double powerOut = controller.calculate(
                getFlywheelVelRadPerSec(), 
                desiredSpeedRadPerSec
            );

        flywheelMotor.set(powerOut);

    }

    public void runShooterPercent(double speed) {

        flywheelMotor.set(speed * powerReduction);

    }

    public void stopShooter() {

        flywheelMotor.set(0);

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
