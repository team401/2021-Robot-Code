package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOChannels;
import frc.robot.Constants.SuperstructureConstants;

public class ConveyorSubsystem extends SubsystemBase {

    private final CANSparkMax conveyorMotor = new CANSparkMax(CANDevices.conveyorMotorId, MotorType.kBrushless);
    
    private final DigitalInput bottomBanner = new DigitalInput(DIOChannels.topBannerPort);
    private final DigitalInput topBanner = new DigitalInput(DIOChannels.bottomBannerPort);

    
    public ConveyorSubsystem() {}

    public Boolean getTopBannerState() {

        return topBanner.get();

    }

    public Boolean getBottomBannerState() {

        return bottomBanner.get();

    }

    public void runConveyor() {

        conveyorMotor.set(SuperstructureConstants.conveyorPower);

    }

    public void stopConveyor() {

        conveyorMotor.set(0);

    }

}
