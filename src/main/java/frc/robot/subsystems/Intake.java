package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SuperstructureConstants;

public class Intake extends SubsystemBase {

    private final TalonSRX intakeMotor = new TalonSRX(CANDevices.intakeMotorId);

    public Intake() {

        intakeMotor.setInverted(true);

    }

    @Override
    public void periodic() {

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
