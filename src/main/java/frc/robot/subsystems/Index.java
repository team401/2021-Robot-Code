package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOChannels;
import frc.robot.Constants.SuperstructureConstants;

public class Index extends SubsystemBase {

    private final VictorSPX conveyorMotor = new VictorSPX(CANDevices.conveyorMotorId);
    
    private final DigitalInput bottomBanner = new DigitalInput(DIOChannels.topBannerPort);
    private final DigitalInput topBanner = new DigitalInput(DIOChannels.bottomBannerPort);
    
    public Index() {

        conveyorMotor.setNeutralMode(NeutralMode.Brake);

    }

    public boolean getTopBannerState() {

        return !topBanner.get();

    }

    public boolean getBottomBannerState() {

        return !bottomBanner.get();

    }

    public void runConveyor() {

        conveyorMotor.set(VictorSPXControlMode.PercentOutput, SuperstructureConstants.conveyorPower);

    }

    public void runJoggingPower() {

        conveyorMotor.set(VictorSPXControlMode.PercentOutput, SuperstructureConstants.jogFowardPower);

    }

    public void stopConveyor() {

        conveyorMotor.set(VictorSPXControlMode.PercentOutput, 0);

    }

    public void reverseConveyor() {

        conveyorMotor.set(VictorSPXControlMode.PercentOutput, -SuperstructureConstants.conveyorPower);

    }

}
