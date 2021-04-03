package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualReverse extends CommandBase {

    private final IntakeSubsystem intake;
    private final IndexingSubsystem indexer;
    private final ShooterSubsystem shooter;

    public ManualReverse(IntakeSubsystem subsystem, IndexingSubsystem index, ShooterSubsystem shoot) {

        intake = subsystem;
        indexer = index;
        shooter = shoot;

        addRequirements(intake, indexer, shooter);

    }
    
}
