package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drivetrain.AlignWithVisionTarget;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.QuickTurn;
import frc.robot.commands.superstructure.shooting.Shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexingSubsystem;

public class InfiniteRecharge2021Auto extends SequentialCommandGroup {

    public enum StartingPosition {
        Left,
        Mid,
        Right
    }

    public enum IntakeSource {
        TrenchLeft,
        Mid, 
        TrenchRight
    }

    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final IndexingSubsystem indexer;
    private final VisionSubsystem limelight;
    private final ShooterSubsystem shooter;

    private final StartingPosition startingPosition;
    private final IntakeSource intakeSource;

    public InfiniteRecharge2021Auto(
        StartingPosition position, 
        IntakeSource source, 
        DriveSubsystem subsystem, 
        IntakeSubsystem intaker, 
        IndexingSubsystem index, 
        VisionSubsystem vision, 
        ShooterSubsystem shoot
    ) {

        drive = subsystem;
        intake = intaker;
        indexer = index;
        limelight = vision;
        shooter = shoot;

        startingPosition = position;
        intakeSource = source;

        addCommands(
            new AlignWithVisionTarget(drive, limelight)
        );

        /*switch (startingPosition) {

            case Left:
                
                switch (intakeSource) {

                    case TrenchLeft:

                        addCommands();

                    case Mid:

                        addCommands();

                }
            
            case Mid:
                
                switch (intakeSource) {

                    case TrenchLeft:

                        addCommands();

                    case Mid:

                        addCommands();

                    case TrenchRight:

                        addCommands();

                }

            case Right:
                
                switch (intakeSource) {

                    case Mid:

                        addCommands();

                    case TrenchRight:

                        addCommands(
                            new AlignWithVisionTarget(drive, limelight)
                        );

                }
        
        }*/

    }

}
