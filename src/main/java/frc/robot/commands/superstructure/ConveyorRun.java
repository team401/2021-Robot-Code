package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorRun extends CommandBase{

    private final ConveyorSubsystem conveyor;

    public ConveyorRun(ConveyorSubsystem subsystem){
        addRequirements(subsystem);
        conveyor = subsystem;
    }

    public void initialize() {
        conveyor.runConveyor();
    }

    public boolean isFinished() {
        return true;
    }
    
}
