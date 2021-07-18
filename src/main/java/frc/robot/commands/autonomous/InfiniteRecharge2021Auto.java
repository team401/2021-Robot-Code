package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class InfiniteRecharge2021Auto extends CommandGroupBase {

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

    private final DriveSubsystem drive = new DriveSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final VisionSubsystem limelight = new VisionSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    private final StartingPosition startingPosition;
    private final IntakeSource intakeSource;

    public InfiniteRecharge2021Auto(StartingPosition position, IntakeSource source) {

        startingPosition = position;
        intakeSource = source;
    
    }

	@Override
	public void addCommands(Command... commands) {

		switch (startingPosition) {

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

                        addCommands();

                }

        }

	}

}
    
