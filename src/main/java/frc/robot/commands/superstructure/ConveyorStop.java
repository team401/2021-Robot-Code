package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorStop extends CommandBase{

    private final ConveyorSubsystem conveyor;

    public ConveyorStop(ConveyorSubsystem subsystem){
        addRequirements(subsystem);
        conveyor = subsystem;
    }

    public void initialize() {
        conveyor.stopConveyor();
    }

    public boolean isFinished() {
        return true;
    }
    
}