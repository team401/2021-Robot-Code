package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.Constants.PneumaticChannels;

public class ClimbingSubsystem extends SubsystemBase {

    private boolean isDeployed = false;
    private boolean isLocked = false;

    private final double leftControllerkP = 0;
    private final double leftControllerkI = 0;
    private final double leftControllerkD = 0;
    private final double rightControllerkP = 0;
    private final double rightControllerkI = 0;
    private final double rightControllerkD = 0;

    private final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0, 0);
    private final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0, 0);

    private final CANSparkMax leftClimberMotor = new CANSparkMax(CANDevices.leftClimberMotorId, MotorType.kBrushless);
    private final CANSparkMax rightClimberMotor = new CANSparkMax(CANDevices.rightClimberMotorId, MotorType.kBrushless);

    private final DoubleSolenoid deploySolenoid = 
        new DoubleSolenoid(
            PneumaticChannels.climberSolenoidChannels[0], 
            PneumaticChannels.climberSolenoidChannels[1]
        );

    private final DoubleSolenoid lockingSolenoid = 
        new DoubleSolenoid(
            PneumaticChannels.lockingSolenoidChannels[0],
            PneumaticChannels.lockingSolenoidChannels[1]
        );

    private final CANPIDController leftController = leftClimberMotor.getPIDController();
    private final CANPIDController rightController = rightClimberMotor.getPIDController();

    private final CANEncoder leftEncoder = leftClimberMotor.getEncoder();
    private final CANEncoder rightEncoder = rightClimberMotor.getEncoder();

    public ClimbingSubsystem() {

        deploySolenoid.set(Value.kReverse);
        lockingSolenoid.set(Value.kReverse);

        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);

        leftController.setP(leftControllerkP);
        leftController.setI(leftControllerkI);
        leftController.setD(leftControllerkD);
        rightController.setP(rightControllerkP);
        rightController.setI(rightControllerkI);
        rightController.setD(rightControllerkD);
        
        leftEncoder.setPositionConversionFactor(ClimbingConstants.winchDiameterInches * Math.PI);
        rightEncoder.setPositionConversionFactor(ClimbingConstants.winchDiameterInches * Math.PI);

        leftEncoder.setVelocityConversionFactor(ClimbingConstants.winchDiameterInches * Math.PI);
        rightEncoder.setVelocityConversionFactor(ClimbingConstants.winchDiameterInches * Math.PI);

    }

    public void resetEncoders() {

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

    }

    public void updateClimberLeft(double desiredPositionLeft) {

        leftController.setReference(
            desiredPositionLeft,
            ControlType.kSmartMotion,
            0,
            leftFF.calculate(desiredPositionLeft)
        );

    }

    public void updateClimberRight(double desiredPositionRight) {

        rightController.setReference(
            desiredPositionRight, 
            ControlType.kSmartMotion,
            0,
            rightFF.calculate(desiredPositionRight)
        );

    }

    public double getCurrentPositionLeft() {

        return leftEncoder.getPosition();

    }

    public double getCurrentPositionRight() {

        return rightEncoder.getPosition();

    }

    public double getCurrentVelocityLeft() {

        return leftEncoder.getVelocity();

    }

    public double getCurrentVelocityRight() {

        return rightEncoder.getVelocity();

    }

    public void deployClimbers() {
    
        deploySolenoid.set(Value.kForward);
        
        isDeployed = true;

    }

    public boolean getIsDeployed() {

        return isDeployed;

    }

    public void lockClimbers() {

        lockingSolenoid.set(Value.kForward);

        isLocked = true;

    }

    public boolean getIsLocked() {

        return isLocked;

    }
    
}
