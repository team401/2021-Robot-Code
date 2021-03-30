package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DIOChannels;
import frc.robot.Constants.SuperstructureConstants;

public class IndexingSubsystem extends SubsystemBase {

    private final WPI_TalonFX conveyorMotor = new WPI_TalonFX(CANDevices.conveyorMotorId);
    
    private final DigitalInput bottomBanner = new DigitalInput(DIOChannels.topBannerPort);
    private final DigitalInput topBanner = new DigitalInput(DIOChannels.bottomBannerPort);
    
    public IndexingSubsystem() {

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

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("topSensorState", getTopBannerState());
        SmartDashboard.putBoolean("bottomSensorState", getBottomBannerState());

    }

}
