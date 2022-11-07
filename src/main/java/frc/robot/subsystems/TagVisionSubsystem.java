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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class TagVisionSubsystem extends SubsystemBase {

  private final PhotonCamera photonCamera;
  private final DriveSubsystem drivetrainSubsystem;

  private static final Pose2d targetPose = new Pose2d(Units.inchesToMeters(84), 0.0, Rotation2d.fromDegrees(180));

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  private static final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01));
  private static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private PhotonPipelineResult previousPipelineResult = new PhotonPipelineResult();

  public TagVisionSubsystem(PhotonCamera photonCamera, DriveSubsystem drivetrainSubsystem) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    poseEstimator = new SwerveDrivePoseEstimator(
        drivetrainSubsystem.getGyroscopeRotation(),
        new Pose2d(),
        DriveConstants.kinematics, stateStdDevs,
        localMeasurementStdDevs, visionMeasurementStdDevs);

    tab.addString("Pose (X, Y)", this::getFomattedPose).withPosition(0, 4);
    tab.addNumber("Pose Degrees", () -> getCurrentPose().getRotation().getDegrees()).withPosition(1, 4);
    tab.add(field2d);
  }

  @Override
  public void periodic() {
    // Update pose estimator with visible targets
    var pipelineResult = photonCamera.getLatestResult();
    if (!pipelineResult.equals(previousPipelineResult) && pipelineResult.hasTargets()) {
      previousPipelineResult = pipelineResult;
      double imageCaptureTime = Timer.getFPGATimestamp() - (pipelineResult.getLatencyMillis() / 1000d);

      for (PhotonTrackedTarget target : pipelineResult.getTargets()) {

        var fiducialId = target.getFiducialId();
        if (fiducialId > -1) {
          Transform3d camToTarget = target.getCameraToTarget();
          Transform2d transform = new Transform2d(
              camToTarget.getTranslation().toTranslation2d(),
              camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));

          Pose2d camPose = targetPose.transformBy(transform.inverse());

          Pose2d visionMeasurement = camPose.transformBy(VisionConstants.CameraToRobot);
          // field2d.getObject("MyRobot" + fiducialId).setPose(visionMeasurement);
          // SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
          // visionMeasurement.getTranslation().getX(),
          // visionMeasurement.getTranslation().getY(),
          // visionMeasurement.getRotation().getDegrees()));
          poseEstimator.addVisionMeasurement(visionMeasurement, imageCaptureTime);
        }
      }
    }
    // Update pose estimator with drivetrain sensors
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModuleStates());

    field2d.setRobotPose(getCurrentPose());
  }

  private String getFomattedPose() {
    Pose2d pose = getCurrentPose();
    return String.format("(%.2f, %.2f)",
        Units.metersToInches(pose.getX()),
        Units.metersToInches(pose.getY()));
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
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
    poseEstimator.resetPosition(newPose, drivetrainSubsystem.getGyroscopeRotation());
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    drivetrainSubsystem.resetRealitivity();
    poseEstimator.resetPosition(
        new Pose2d(), drivetrainSubsystem.getGyroscopeRotation());
  }

	

}