package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PneumaticChannels;
import frc.robot.Constants.SuperstructureConstants;

public class IntakeSubsystem extends SubsystemBase {

    /**
     * Subsystem that controls the intake of the robot
     */


    private final TalonSRX intakeMotor = new TalonSRX(CANDevices.intakeMotorId);

    private final DoubleSolenoid intakeSolenoid = 
        new DoubleSolenoid(
            PneumaticChannels.intakeSolenoidChannels[0], 
            PneumaticChannels.intakeSolenoidChannels[1]
        );

    public IntakeSubsystem() {

        intakeMotor.setInverted(true);

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

    public void extendIntake() {

        intakeSolenoid.set(Value.kForward);

    }

    public void retractIntake() {

        intakeSolenoid.set(Value.kReverse);

    }

    public void toggleIntake() {

        if (intakeSolenoid.get() == Value.kForward) intakeSolenoid.set(Value.kReverse);
        else intakeSolenoid.set(Value.kForward);

    }

}
