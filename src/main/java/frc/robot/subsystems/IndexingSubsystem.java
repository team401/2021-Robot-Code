package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOChannels;
import frc.robot.Constants.SuperstructureConstants;

public class IndexingSubsystem extends SubsystemBase {

    /**
     * Subsystem for the indexing, not including the intake or kicker
     * 
     */

    private final WPI_TalonFX conveyorMotor = new WPI_TalonFX(CANDevices.conveyorMotorId);
    
    //banner sensors
    private final DigitalInput bottomBanner = new DigitalInput(DIOChannels.topBannerPort);
    private final DigitalInput topBanner = new DigitalInput(DIOChannels.bottomBannerPort);
    
    public IndexingSubsystem() {

        //ensure the intake motor stops when we don't command it to prevent jamming
        conveyorMotor.setNeutralMode(NeutralMode.Brake);

    }

    public boolean getTopBannerState() {

        return !topBanner.get();

    }

    public boolean getBottomBannerState() {

        return !bottomBanner.get();

    }

    public void runConveyor() {

        conveyorMotor.set(SuperstructureConstants.conveyorPower);

    }

    public void runJoggingPower() {

        conveyorMotor.set(SuperstructureConstants.jogFowardPower);

    }

    public void stopConveyor() {

        conveyorMotor.set(0);

    }

    public void reverseConveyor() {

        conveyorMotor.set(-SuperstructureConstants.conveyorPower);

    }

}
