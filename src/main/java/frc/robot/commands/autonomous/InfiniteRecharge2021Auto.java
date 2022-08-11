package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoTrajectories;
import frc.robot.commands.drivetrain.AlignWithVisionTarget;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.superstructure.indexing.Waiting;
import frc.robot.commands.superstructure.shooting.RampUpToSpeed;
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
        Generator, 
        TrenchRight,
        NoSource
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

                    case NoSource:

                        drive.resetPose(AutoTrajectories.startLeftDriveOffInitLine.getInitialPose());

                        addCommands(
                            new ParallelCommandGroup(
                                new AlignWithVisionTarget(drive, limelight).andThen(new InstantCommand(() -> drive.drive(0, 0, 0, false))),
                                new SequentialCommandGroup(
                                    new RampUpToSpeed(Units.rotationsPerMinuteToRadiansPerSecond(4100), shooter),
                                    new Shoot(shooter, indexer)
                                )
                            ),
                            new FollowTrajectory(drive, AutoTrajectories.startLeftDriveOffInitLine)
                        );

                        break;
                    default:
                        break;

                }

            case Mid:
                
                switch (intakeSource) {

                    case Generator:
                        
                        drive.resetPose(new Pose2d(Units.inchesToMeters(510), Units.inchesToMeters(162), new Rotation2d(0)));

                        addCommands();

                        break;

                    case TrenchRight:

                        drive.resetPose(AutoTrajectories.startMidToTrenchRight.getInitialPose());

                        addCommands(
                            new ParallelCommandGroup(
                                new AlignWithVisionTarget(drive, limelight).andThen(new InstantCommand(() -> drive.drive(0, 0, 0, false))),
                                new SequentialCommandGroup(
                                    new RampUpToSpeed(Units.rotationsPerMinuteToRadiansPerSecond(4100), shooter),
                                    new Shoot(shooter, indexer)
                                )
                            ),
                            new InstantCommand(intake::runIntakeMotor),
                            new ParallelDeadlineGroup(
                                new FollowTrajectory(
                                    drive, 
                                    AutoTrajectories.startMidToTrenchRight
                                ), 
                                new Waiting(indexer)
                            ),
                            new InstantCommand(intake::stopIntakeMotor),
                            new FollowTrajectory(
                                drive,
                                AutoTrajectories.trenchRightToShootRight
                            ),
                            new ParallelCommandGroup(
                                new AlignWithVisionTarget(drive, limelight).andThen(new InstantCommand(() -> drive.drive(0, 0, 0, false))),
                                new SequentialCommandGroup(
                                    new RampUpToSpeed(Units.rotationsPerMinuteToRadiansPerSecond(4100), shooter),
                                    new Shoot(shooter, indexer)
                                )
                            )
                        );

                        break;

                    case NoSource:

                        drive.resetPose(AutoTrajectories.startMidDriveOffInitLine.getInitialPose());

                        addCommands(
                            new ParallelCommandGroup(
                                new AlignWithVisionTarget(drive, limelight).andThen(new InstantCommand(() -> drive.drive(0, 0, 0, false))),
                                new SequentialCommandGroup(
                                new RampUpToSpeed(Units.rotationsPerMinuteToRadiansPerSecond(4100), shooter),
                                new Shoot(shooter, indexer)
                                )
                            ),
                            new FollowTrajectory(drive, AutoTrajectories.startMidDriveOffInitLine)
                        );

                        break;

                }

            case Right:
                
                switch (intakeSource) {

                    case Generator:

                        drive.resetPose(AutoTrajectories.startRightToGenerator.getInitialPose());

                        addCommands(); 

                        break;

                    case TrenchRight:

                        drive.resetPose(AutoTrajectories.startRightToTrenchRight.getInitialPose());

                        addCommands(
                            new ParallelCommandGroup(
                                new AlignWithVisionTarget(drive, limelight).andThen(new InstantCommand(() -> drive.drive(0, 0, 0, false))),
                                new SequentialCommandGroup(
                                    new RampUpToSpeed(Units.rotationsPerMinuteToRadiansPerSecond(4100), shooter),
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
                                new AlignWithVisionTarget(drive, limelight).andThen(new InstantCommand(() -> drive.drive(0, 0, 0, false))),
                                new SequentialCommandGroup(
                                new RampUpToSpeed(Units.rotationsPerMinuteToRadiansPerSecond(4100), shooter),
                                new Shoot(shooter, indexer)
                                )
                            )

                        );

                        break;

                    case NoSource:

                        drive.resetPose(AutoTrajectories.startRightDriveOffInitLine.getInitialPose());

                        addCommands(
                            /*new ParallelCommandGroup(
                                new AlignWithVisionTarget(drive, limelight).andThen(new InstantCommand(() -> drive.drive(0, 0, 0, false))),
                                new SequentialCommandGroup(
                                    new RampUpToSpeed(Units.rotationsPerMinuteToRadiansPerSecond(4100), shooter),
                                    new Shoot(shooter, indexer)
                                )
                            ),*/
                            new FollowTrajectory(drive, AutoTrajectories.startRightDriveOffInitLine)
                        );

                        break;

                }
        
        }

    }

}
