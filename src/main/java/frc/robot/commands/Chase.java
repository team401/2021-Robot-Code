package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TagVisionSubsystem;

public class Chase extends CommandBase {

	private final TagVisionSubsystem vision;
	private final DriveSubsystem drive;

	private static final Pose2d tagToDesired = new Pose2d(0.0, 10.0, new Rotation2d());

	private final PIDController xController = new PIDController(0, 0, 0);
	private final PIDController yController = new PIDController(0, 0, 0);
	private final PIDController thetaController = new PIDController(0, 0, 0);

	public Chase(TagVisionSubsystem vision, DriveSubsystem drive) {
		this.vision = vision;
		this.drive = drive;

		xController.setTolerance(0.1);
		yController.setTolerance(0.1);
		thetaController.setTolerance(0.1);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		if (vision.hasTarget()) {
			double xOutput = xController.calculate(vision.getCurrentPose().getX(), tagToDesired.getX());
			double yOutput = yController.calculate(vision.getCurrentPose().getY(), tagToDesired.getY());
			double thetaOutput = thetaController.calculate(vision.getCurrentPose().getRotation().getDegrees(), tagToDesired.getRotation().getDegrees());

			drive.drive(xOutput, yOutput, thetaOutput, false);
		}
		else {
			drive.stop();
		}
	}
	
	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}