/*package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignWithVisionTarget extends CommandBase {
    
    private final DriveSubsystem drive;
    private final Limelight limelight;

    private final PIDController controller = new PIDController(0.5, 0, 0);

    public AlignWithVisionTarget(DriveSubsystem drive, Limelight limelight) {

        this.drive = drive;
        this.limelight = limelight;

    }

    @Override
    public void execute() {

        Rotation2d tx = Rotation2d.fromDegrees(limelight.getTx());

        double rotationOut = controller.calculate(tx.getRadians(), );

    }

}
*/