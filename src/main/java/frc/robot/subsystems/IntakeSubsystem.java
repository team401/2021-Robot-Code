package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PneumaticChannels;
import frc.robot.Constants.SuperstructureConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax intakeMotor = new CANSparkMax(CANDevices.intakeMotorId, MotorType.kBrushless);

    private final DoubleSolenoid leftIntakeSolenoid = 
        new DoubleSolenoid(
            PneumaticChannels.leftIntakeSolenoidChannels[0], 
            PneumaticChannels.leftIntakeSolenoidChannels[1]
        );
    
    private final DoubleSolenoid rightIntakeSolenoid = 
        new DoubleSolenoid(
            PneumaticChannels.rightIntakeSolenoidChannels[0], 
            PneumaticChannels.rightIntakeSolenoidChannels[1]
        );

    public IntakeSubsystem() {

        intakeMotor.setInverted(true);

    }

    public void runIntakeMotor() {

        intakeMotor.set(SuperstructureConstants.intakingPower);

    }

    public void stopIntakeMotor() {

        intakeMotor.set(0);

    }

    public void extendIntake() {

        leftIntakeSolenoid.set(Value.kForward);
        rightIntakeSolenoid.set(Value.kForward);

    }

    public void retractIntake() {

        leftIntakeSolenoid.set(Value.kReverse);
        rightIntakeSolenoid.set(Value.kReverse);

    }

}
