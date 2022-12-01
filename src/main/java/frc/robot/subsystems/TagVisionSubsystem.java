package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants;
import static frc.robot.Constants.DriveConstants;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class TagVisionSubsystem extends SubsystemBase {

  private final PhotonCamera photonCamera;
  private final DriveSubsystem drivetrainSubsystem;

  private static final Pose2d targetPose = new Pose2d(Units.inchesToMeters(84), 0.0, Rotation2d.fromDegrees(180));

  private final Field2d field2d = new Field2d();

  private PhotonPipelineResult previousPipelineResult = new PhotonPipelineResult();

  public TagVisionSubsystem(PhotonCamera photonCamera, DriveSubsystem drivetrainSubsystem) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  @Override
  public void periodic() {
    // Update pose estimator with visible targets
    var pipelineResult = photonCamera.getLatestResult();
    previousPipelineResult = pipelineResult;
    
    if (!pipelineResult.equals(previousPipelineResult) && pipelineResult.hasTargets()) {
      double imageCaptureTime = Timer.getFPGATimestamp() - (pipelineResult.getLatencyMillis() / 1000d);

      PhotonTrackedTarget target = pipelineResult.getBestTarget();

      var fiducialId = target.getFiducialId();
      if (fiducialId == -1) {
        return;
      }

      Transform3d camToTarget3d = target.getCameraToTarget();

      Transform2d camToTarget = new Transform2d(
          camToTarget3d.getTranslation().toTranslation2d(),
          camToTarget3d.getRotation().toRotation2d());

      Pose2d camPose = targetPose.transformBy(camToTarget.inverse());

      Pose2d visionMeasurement = camPose.transformBy(VisionConstants.CameraToRobot);
      
      SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
        visionMeasurement.getTranslation().getX(),
        visionMeasurement.getTranslation().getY(),
        visionMeasurement.getRotation().getDegrees()));
      
      }
    SmartDashboard.putBoolean("Has Target", hasTarget());
    getCurrentPose();
  }
  

  private String getFomattedPose() {
    Pose2d pose = getCurrentPose();
    return String.format("(%.2f, %.2f)",
        Units.metersToInches(pose.getX()),
        Units.metersToInches(pose.getY()));
  }

  public Pose2d getCurrentPose() {
    Transform3d cameraToTag = getCurrentTarget().getCameraToTarget();

    Pose2d pose = new Pose2d(cameraToTag.getX(), cameraToTag.getY(), cameraToTag.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(180)));

    SmartDashboard.putNumber("Tag Pose X", pose.getX());
    SmartDashboard.putNumber("Tag Pose Y", pose.getY());
    SmartDashboard.putNumber("Tag Pose theta", pose.getRotation().getDegrees());
    
    return pose;
  }

	public PhotonTrackedTarget getCurrentTarget() {
		// Running getBestTarget() on an empty pipeline result gives a warning and returns null.
		// Part of the purpose of this subsystem should probably be to avoid all the nulls used by Photonlib.
		return previousPipelineResult.hasTargets() ? previousPipelineResult.getBestTarget() : new PhotonTrackedTarget();
	}

	public boolean hasTarget() {
		return previousPipelineResult.hasTargets();
	}

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    drivetrainSubsystem.resetRealitivity();
    
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    drivetrainSubsystem.resetRealitivity();
    
  }

	

}