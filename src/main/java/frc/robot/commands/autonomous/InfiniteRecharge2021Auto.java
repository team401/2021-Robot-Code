package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutoTrajectories;
import frc.robot.commands.drivetrain.AlignWithVisionTarget;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.QuickTurn;
import frc.robot.commands.superstructure.indexing.Waiting;
import frc.robot.commands.superstructure.shooting.RampUpToSpeed;
import frc.robot.commands.superstructure.shooting.RampUpWithVision;
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

    private final StartingPosition startingPosition;
    private final IntakeSource intakeSource;

    public InfiniteRecharge2021Auto(
        StartingPosition position, 
        IntakeSource source, 
        DriveSubsystem drive, 
        IntakeSubsystem intake, 
        IndexingSubsystem indexer, 
        VisionSubsystem limelight, 
        ShooterSubsystem shooter
    ) {

        startingPosition = position;
        intakeSource = source;

        drive.resetImu();

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

                        drive.resetPose(new Pose2d(Units.inchesToMeters(510), Units.inchesToMeters(162), new Rotation2d(0)));

                        addCommands(
                            new ParallelCommandGroup(
                                new AlignWithVisionTarget(drive, limelight),
                                new SequentialCommandGroup(
                                    new RampUpToSpeed(Units.rotationsPerMinuteToRadiansPerSecond(4250), shooter),
                                    new Shoot(shooter, indexer)
                                )
                            ),
                            new InstantCommand(intake::runIntakeMotor),
                            new FollowTrajectory(
                                drive, 
                                AutoTrajectories.startMidToTrenchRight
                            ),
                            new InstantCommand(intake::stopIntakeMotor),
                            new FollowTrajectory(
                                drive,
                                AutoTrajectories.trenchRightToShootRight
                            ),
                            new ParallelCommandGroup(
                                new AlignWithVisionTarget(drive, limelight),
                                new SequentialCommandGroup(
                                    new RampUpToSpeed(Units.rotationsPerMinuteToRadiansPerSecond(4250), shooter),
                                    new Shoot(shooter, indexer)
                                )
                            )
                        );

                }

            case Right:
                
                switch (intakeSource) {

                    case Mid:

                        addCommands(); 

                    case TrenchRight:

                        drive.resetPose(new Pose2d(Units.inchesToMeters(510), Units.inchesToMeters(48), new Rotation2d(0)));

                        addCommands(
                            new ParallelCommandGroup(
                                new AlignWithVisionTarget(drive, limelight),
                                new SequentialCommandGroup(
                                    new RampUpToSpeed(Units.rotationsPerMinuteToRadiansPerSecond(4250), shooter),
                                    new Shoot(shooter, indexer)
                                )
                            ),
                            new InstantCommand(intake::runIntakeMotor),
                            new FollowTrajectory(
                                drive, 
                                AutoTrajectories.startRightToTrenchRight
                            ),
                            new InstantCommand(intake::stopIntakeMotor),
                            new FollowTrajectory(
                                drive,
                                AutoTrajectories.trenchRightToShootRight
                            ),
                            new ParallelCommandGroup(
                                new AlignWithVisionTarget(drive, limelight),
                                new SequentialCommandGroup(
                                    new RampUpToSpeed(Units.rotationsPerMinuteToRadiansPerSecond(4250), shooter),
                                    new Shoot(shooter, indexer)
                                )
                            )
                        );

                }
        
        }

    }

}
