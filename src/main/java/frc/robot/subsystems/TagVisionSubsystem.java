package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class TagVisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;

    private final Field2d field2d = new Field2d();

    private Pose2d latestPose;
    private PhotonPipelineResult latestResult;
    private double latency;

    public TagVisionSubsystem(PhotonCamera camera) {
        this.camera = camera;

        latestPose = new Pose2d();
        latestResult = null;
        latency = 100000000.0;
    }

    @Override
    public void periodic() {
        PhotonPipelineResult newResult = camera.getLatestResult();

        SmartDashboard.putBoolean("Can See Target", canSeeTag());

        if (newResult.equals(latestResult)) {
            return;
        }

        if (!newResult.hasTargets()) {
            return;
        }

        latestResult = newResult;

        PhotonTrackedTarget target = latestResult.getBestTarget();
        // if (target.getPoseAmbiguity() > 0.2 || target.getPoseAmbiguity() < 0) {
        //     return;
        // }
        
        Transform3d cameraToTarget3d = target.getCameraToTarget();
        Transform2d cameraToTarget2d = new Transform2d(cameraToTarget3d.getTranslation().toTranslation2d(), cameraToTarget3d.getRotation().toRotation2d());

        Transform2d targetToRobot = cameraToTarget2d.inverse();

        int fiducialId = target.getFiducialId();
        // if (fiducialId == 0) {
        //     fieldToTarget = FieldConstants.rightTagLocation;
        // }
        // if (fiducialId == 7) {
        //     fieldToTarget = FieldConstants.leftTagLocation;
        // }
        Pose2d fieldToTarget = FieldConstants.fieldToTarget;

        Pose2d fieldToRobot = fieldToTarget.transformBy(targetToRobot);

        latestPose = fieldToRobot;
        latency = latestResult.getLatencyMillis();

        field2d.setRobotPose(latestPose);
        SmartDashboard.putNumber("Vision X", latestPose.getX());
        SmartDashboard.putNumber("Vision Y", latestPose.getY());
    }

    public Pose2d whereIsRobot() {
        return latestPose;
    }

    public double getLatencyMillis() {
        return latency;
    }

    public boolean canSeeTag() {
        return latestResult == null ? false : latestResult.hasTargets();
    }
}