package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SuperstructureConstants;

public class IntakeSubsystem extends SubsystemBase {

    /**
     * Subsystem that controls the intake of the robot
     */


    private final TalonSRX intakeMotor = new TalonSRX(CANDevices.intakeMotorId);

    public IntakeSubsystem() {

        intakeMotor.setInverted(false);

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("intake motor measured current", intakeMotor.getMotorOutputPercent());

    }

    public void runIntakeMotor() {

        intakeMotor.set(TalonSRXControlMode.PercentOutput, SuperstructureConstants.intakingPower);

    }

    public void stopIntakeMotor() {

        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);

    }

    public void reverseIntakeMotor() {

        intakeMotor.set(TalonSRXControlMode.PercentOutput, -SuperstructureConstants.intakingPower);

    }

}
