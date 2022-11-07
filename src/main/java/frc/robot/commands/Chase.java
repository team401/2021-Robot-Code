package frc.robot.commands;

import static frc.robot.Constants.VisionConstants;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TagVisionSubsystem;

public class Chase extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = 
      new TrapezoidProfile.Constraints(8, 8);

  private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180.0));

  private final DriveSubsystem drive;
  private final TagVisionSubsystem vision;

  private final ProfiledPIDController xController = new ProfiledPIDController(4, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(4, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRATINTS);

  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;

  public Chase(DriveSubsystem drive, TagVisionSubsystem vision) {
    this.drive = drive;
    this.vision = vision;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-1, 1);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    goalPose = null;
    lastTarget = null;
    Pose2d robotPose = vision.getCurrentPose();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    Pose2d robotPose = vision.getCurrentPose();
    PhotonTrackedTarget target = vision.getCurrentTarget();
    if (target.getArea() > 0) {
      if (!target.equals(lastTarget)) {
        // This is new target data, so recalculate the goal
        lastTarget = target;

        // Get the transformation from the camera to the tag (in 2d)
        var camToTarget = target.getCameraToTarget();
        var transform = new Transform2d(
          camToTarget.getTranslation().toTranslation2d(),
          camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));
          
          // Transform the robot's pose to find the tag's pose
          var cameraPose = robotPose.transformBy(VisionConstants.CameraToRobot.inverse());
          Pose2d targetPose = cameraPose.transformBy(transform);
          
          // Transform the tag's pose to set our goal
          goalPose = targetPose.transformBy(TAG_TO_GOAL);
      }

      if (goalPose != null) {
        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
      }
    }
    
    double xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }
    
    double ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
    if (omegaController.atGoal()) {
      omegaSpeed = 0;
    }

    drive.drive(xSpeed, ySpeed, omegaSpeed, true);
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(0.0, 0.0, 0.0, true);
  }

}