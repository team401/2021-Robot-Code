package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    /**
     * Subsystem that controls the drivetrain of the robot
     * Handles all the odometry and base movement for the chassis
     */

    /**
     * absolute encoder offsets for the wheels
     * 180 degrees added to offset values to invert one side of the robot so that it doesn't spin in place
     */
    private static final double frontLeftAngleOffset = Units.degreesToRadians(239.5);
    private static final double frontRightAngleOffset = Units.degreesToRadians(256.7 + 180);
    private static final double rearLeftAngleOffset = Units.degreesToRadians(322.8);
    private static final double rearRightAngleOffset = Units.degreesToRadians(180.0 + 180);

    /**
     * SwerveModule objects
     * Parameters:
     * drive motor can ID
     * rotation motor can ID
     * external CANCoder can ID
     * measured CANCoder offset
     */

    private final SwerveModule frontLeft = 
        new SwerveModule(
            CANDevices.frontLeftDriveMotorId,
            CANDevices.frontLeftRotationMotorId,
            CANDevices.frontLeftRotationEncoderId,
            frontLeftAngleOffset
        );

    private final SwerveModule frontRight = 
        new SwerveModule(
            CANDevices.frontRightDriveMotorId,
            CANDevices.frontRightRotationMotorId,
            CANDevices.frontRightRotationEncoderId,
            frontRightAngleOffset
        );

    private final SwerveModule rearLeft = 
        new SwerveModule(
            CANDevices.rearLeftDriveMotorId,
            CANDevices.rearLeftRotationMotorId,
            CANDevices.rearLeftRotationEncoderId,
            rearLeftAngleOffset
        );

    private final SwerveModule rearRight = 
        new SwerveModule(
            CANDevices.rearRightDriveMotorId,
            CANDevices.rearRightRotationMotorId,
            CANDevices.rearRightRotationEncoderId,
            rearRightAngleOffset
        );

    // commanded values from the joysticks and field relative value to use in AlignWithTargetVision and AlignWithGyro
    private double commandedForward = 0;
    private double commandedStrafe = 0;
    private double commandedRotation = 0;

    private boolean isCommandedFieldRelative = false;

    private final PigeonIMU imu = new PigeonIMU(CANDevices.imuId);

    private boolean driveToggled = true;

    /**
     * odometry for the robot, measured in meters for linear motion and radians for rotational motion
     * Takes in kinematics and robot angle for parameters
     */
    private final SwerveDriveOdometry odometry = 
        new SwerveDriveOdometry(
            DriveConstants.kinematics, 
            new Rotation2d(getHeading().getRadians())
        );

    public DriveSubsystem() {

        imu.setYaw(0.0);

        // initialize the rotation offsets for the CANCoders
        frontLeft.initRotationOffset();
        frontRight.initRotationOffset();
        rearLeft.initRotationOffset();
        rearRight.initRotationOffset();

        // reset the measured distance driven for each module
        frontLeft.resetDistance();
        frontRight.resetDistance();
        rearLeft.resetDistance();
        rearRight.resetDistance();

        SmartDashboard.putNumber("Max Drive Speed ft s", DriveConstants.maxDriveSpeed);
        SmartDashboard.putNumber("Turn Rate deg s", DriveConstants.teleopTurnRateDegPerSec);

    }

    @Override
    public void periodic() {

        // update the odometry every 20ms
        odometry.update(getHeading(), getModuleStates());

        SmartDashboard.putNumber("heading", getHeading().getDegrees());
        SmartDashboard.putNumber("Odometry x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry y", odometry.getPoseMeters().getY());

        SmartDashboard.putBoolean("Drive Enabled", driveToggled);
        
    }
    
    /**
     * Drives the robot if drive is toggled. If not, does nothing
     * @param forward speed forward in m/s: linear value
     * @param strafe speed sideways in m/s; linear value
     * @param rotation rotation speed (counter-clockwise) in radians/s
     * @param isFieldRelative if the control is field relative or robot relative
     */
    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

        if (!driveToggled)
            return;

        // update the drive inputs for use in AlignWithGyro and AlignWithTargetVision control
        commandedForward = forward;
        commandedStrafe = strafe;
        commandedRotation = rotation;

        isCommandedFieldRelative = isFieldRelative;

        /**
         * ChassisSpeeds object to represent the overall state of the robot
         * ChassisSpeeds takes a forward and sideways linear value and a rotational value
         * 
         * speeds is set to field relative or default (robot relative) based on parameter
         */
        ChassisSpeeds speeds =
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    forward, strafe, rotation, getHeading())
                : new ChassisSpeeds(forward, strafe, rotation);
        
        // use kinematics (wheel placements) to convert overall robot state to array of individual module states
        SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);

        // make sure the wheels don't try to spin faster than the maximum speed possible
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxDriveSpeed);

        setModuleStates(states);

    }

    /**
     * Method to set the desired state for each swerve module
     * Uses PID and feedforward control to control the linear and rotational values for the modules
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {

        frontLeft.setDesiredStateClosedLoop(moduleStates[0]);
        frontRight.setDesiredStateClosedLoop(moduleStates[1]);
        rearLeft.setDesiredStateClosedLoop(moduleStates[2]);
        rearRight.setDesiredStateClosedLoop(moduleStates[3]);

    }

    // returns an array of SwerveModuleState
    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = {
            new SwerveModuleState(frontRight.getCurrentVelocityMetersPerSecond(), frontRight.getCanEncoderAngle()),
            new SwerveModuleState(rearRight.getCurrentVelocityMetersPerSecond(), rearRight.getCanEncoderAngle()),
            new SwerveModuleState(frontLeft.getCurrentVelocityMetersPerSecond(), frontLeft.getCanEncoderAngle()),
            new SwerveModuleState(rearLeft.getCurrentVelocityMetersPerSecond(), rearLeft.getCanEncoderAngle())
        };

        return states;

    }

    /**
     * Return the current position of the robot on field
     * Based on drive encoder and gyro reading
     */
    public Pose2d getPose() {

        return odometry.getPoseMeters();

    }

    // reset the current pose to a desired pose
    public void resetPose(Pose2d pose) {

        imu.setYaw(0);
        odometry.resetPosition(pose, getHeading());

    }

    // reset the measured distance driven for each module
    public void resetDriveDistances() {

        frontLeft.resetDistance();
        frontRight.resetDistance();
        rearLeft.resetDistance();
        rearRight.resetDistance();

    }

    // return the average distance driven for each module to get an overall distance driven by the robot
    public double getAverageDriveDistanceRadians() {

        return ((
            Math.abs(frontLeft.getDriveDistanceRadians())
            + Math.abs(frontRight.getDriveDistanceRadians())
            + Math.abs(rearLeft.getDriveDistanceRadians())
            + Math.abs(rearRight.getDriveDistanceRadians())) / 4.0);

    }

    // return the average velocity for each module to get an overall velocity for the robot
    public double getAverageDriveVelocityRadiansPerSecond() {

        return ((
            Math.abs(frontLeft.getCurrentVelocityRadiansPerSecond())
            + Math.abs(frontRight.getCurrentVelocityRadiansPerSecond()) 
            + Math.abs(rearLeft.getCurrentVelocityRadiansPerSecond()) 
            + Math.abs(rearRight.getCurrentVelocityRadiansPerSecond())) / 4.0);

    }

    // get the current heading of the robot based on the gyro
    public Rotation2d getHeading() {

        double[] ypr = new double[3];

        imu.getYawPitchRoll(ypr);

        return Rotation2d.fromDegrees(ypr[0]);

    }

    public double[] getCommandedDriveValues() {

        double[] values = {commandedForward, commandedStrafe, commandedRotation};

        return values;

    }

    public boolean getIsFieldRelative() {

        return isCommandedFieldRelative;

    }

    public void resetRealitivity() {
        imu.setYaw(0);
    }

    // Will only drive if the toggle is true. Otherwise will stop moving and refuse to drive
    public void toggleDrive(boolean toggle) {
        if (!toggle)
            drive(0.0, 0.0, 0.0, true);

        driveToggled = toggle;
    }
}