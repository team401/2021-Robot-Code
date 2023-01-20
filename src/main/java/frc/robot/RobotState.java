package frc.robot;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.GeomUtil;
import frc.robot.util.PoseHistory;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private Pose2d fieldToBoot = new Pose2d();
    private final PoseHistory bootToVehicle = new PoseHistory(100);
    private ChassisSpeeds vehicleVelocity = new ChassisSpeeds();

    private RobotState() {
        bootToVehicle.insert(0.0, new Pose2d());
    }

    private static double[] poseToDoubleArray(Pose2d pose) {
        return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
    }

    public void forceRobotPose(Pose2d fieldToVehicle) {
        Pose2d bootToVehicle = this.bootToVehicle.getLatest().orElseThrow().getPose();
        Pose2d vehicleToBoot = GeomUtil.poseInverse(bootToVehicle);
        fieldToBoot = fieldToVehicle.transformBy(GeomUtil.poseToTransform(vehicleToBoot));
    }

    public void recordOdometryObservations(Pose2d bootToVehicle, ChassisSpeeds velocity) {
        this.bootToVehicle.insert(Timer.getFPGATimestamp(), bootToVehicle);
        vehicleVelocity = velocity;
    }

    public Pose2d getFieldToVehicle(double timestamp) {
        Transform2d bootToVehicle = GeomUtil.poseToTransform(this.bootToVehicle.get(timestamp).orElseThrow());
        return fieldToBoot.transformBy(bootToVehicle);
    }

    public Pose2d getLatestFieldToVehicle() {
        Transform2d bootToVehicle = GeomUtil.poseToTransform(this.bootToVehicle.getLatest().orElseThrow().getPose());
        return fieldToBoot.transformBy(bootToVehicle);
    }

    public Pose2d getPredictedFieldToVehicle(double lookaheadTime, double angularLookaheadTime) {
        return getLatestFieldToVehicle().exp(
                new Twist2d(vehicleVelocity.vxMetersPerSecond * lookaheadTime,
                        vehicleVelocity.vyMetersPerSecond * lookaheadTime,
                        vehicleVelocity.omegaRadiansPerSecond * angularLookaheadTime));
    }

    public ChassisSpeeds getVehicleVelocity() {
        return vehicleVelocity;
    }

}