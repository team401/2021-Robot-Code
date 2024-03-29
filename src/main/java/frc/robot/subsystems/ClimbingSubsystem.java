package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.Constants.PneumaticChannels;

public class ClimbingSubsystem extends SubsystemBase {

    private boolean isDeployed = false;

    private final double leftControllerkP = 4;
    private final double leftControllerkI = 0;
    private final double leftControllerkD = 0;
    private final double rightControllerkP = 4;
    private final double rightControllerkI = 0;
    private final double rightControllerkD = 0;

    private final CANSparkMax leftClimberMotor = new CANSparkMax(CANDevices.leftClimberMotorId, MotorType.kBrushless);
    private final CANSparkMax rightClimberMotor = new CANSparkMax(CANDevices.rightClimberMotorId, MotorType.kBrushless);

    private final Solenoid deploySolenoid = new Solenoid(PneumaticChannels.lockingSolenoidChannel);

    private final CANPIDController leftController = leftClimberMotor.getPIDController();
    private final CANPIDController rightController = rightClimberMotor.getPIDController();

    private final CANEncoder leftEncoder = leftClimberMotor.getEncoder();
    private final CANEncoder rightEncoder = rightClimberMotor.getEncoder();

    public ClimbingSubsystem() {

        rightClimberMotor.setInverted(false);
        leftClimberMotor.setInverted(true);

        deploySolenoid.set(false);

        leftClimberMotor.setIdleMode(IdleMode.kCoast);
        rightClimberMotor.setIdleMode(IdleMode.kCoast);

        leftController.setP(leftControllerkP);
        leftController.setI(leftControllerkI);
        leftController.setD(leftControllerkD);
        rightController.setP(rightControllerkP);
        rightController.setI(rightControllerkI);
        rightController.setD(rightControllerkD);
        
        leftEncoder.setPositionConversionFactor(ClimbingConstants.winchDiameterInches * Math.PI * ClimbingConstants.climberGearRatio);
        rightEncoder.setPositionConversionFactor(ClimbingConstants.winchDiameterInches * Math.PI * ClimbingConstants.climberGearRatio);

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("climber left value", getCurrentPositionLeft());
        SmartDashboard.putNumber("climber right value", getCurrentPositionRight());

    }

    public void resetEncoders() {

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

    }

    public void updateClimberLeft(double desiredPositionLeft) {

        leftController.setReference(
            desiredPositionLeft,
            ControlType.kPosition,
            0
        );

    }

    public void updateClimberRight(double desiredPositionRight) {

            rightController.setReference(
                desiredPositionRight, 
                ControlType.kPosition,
                0        
            );

    }

    public void actuateClimberPercentLeft(double percent) {

        leftClimberMotor.set(percent);

    }

    public void actuateClimberPercentRight(double percent) {

        rightClimberMotor.set(percent);

    }

    public double getCurrentPositionLeft() {

        return leftEncoder.getPosition();

    }

    public double getCurrentPositionRight() {

        return rightEncoder.getPosition();

    }

    public void deployClimbers() {
    
        deploySolenoid.set(true);
        
        isDeployed = true;

    }

    public void lockClimbers() {

        deploySolenoid.set(false);

        isDeployed = false;

    }

    public boolean getIsDeployed() {

        return isDeployed;

    }
    
}