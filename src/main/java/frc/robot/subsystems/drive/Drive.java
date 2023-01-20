package frc.robot.subsystems.drive;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.Constants.CANDevices;

public class Drive extends SubsystemBase {

    private final DriveModule[] driveModules = new DriveModule[4];
    private final DriveAngle driveAngle = new DriveAngle();

    private SwerveModuleState moduleStates[] = new SwerveModuleState[4];
    private SwerveModulePosition modulePositions[] = new SwerveModulePosition[4];

    private SwerveDriveOdometry odometry;

    private final Field2d field2d = new Field2d();

    public Drive() {

        driveModules[0] = new DriveModule(CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftRotationMotorID,
            CANDevices.frontLeftRotationEncoderID, DriveConstants.frontLeftAngleOffset);
        driveModules[1] = new DriveModule(CANDevices.frontRightDriveMotorID, CANDevices.frontRightRotationMotorID,
            CANDevices.frontRightRotationEncoderID, DriveConstants.frontRightAngleOffset);
        driveModules[2] = new DriveModule(CANDevices.backLeftDriveMotorID, CANDevices.backLeftRotationMotorID,
            CANDevices.backLeftRotationEncoderID, DriveConstants.backLeftAngleOffset);
        driveModules[3] = new DriveModule(CANDevices.backRightDriveMotorID, CANDevices.backRightRotationMotorID,
            CANDevices.backRightRotationEncoderID, DriveConstants.backRightAngleOffset);

        for (int i = 0; i < 4; i++) {
            driveModules[i].zeroEncoders();
            modulePositions[i] = new SwerveModulePosition();
            moduleStates[i] = new SwerveModuleState();
        }

        for (int i = 0; i < 4; i++) {
            modulePositions[i].distanceMeters = driveModules[i].getDrivePosition() * DriveConstants.wheelRadiusM;
            modulePositions[i].angle = new Rotation2d(driveModules[i].getRotationPosition());
        }
        odometry = new SwerveDriveOdometry(DriveConstants.kinematics, new Rotation2d(), modulePositions, new Pose2d());

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Pitch", driveAngle.getPitch());
        SmartDashboard.putNumber("Roll", driveAngle.getRoll());

        SmartDashboard.putNumber("Yaw", MathUtil.angleModulus(getRotation().getRadians()));

        // Record states
        Rotation2d headingRotation = new Rotation2d(MathUtil.angleModulus(driveAngle.getHeading()));
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = driveModules[i].getModuleState();
            modulePositions[i].distanceMeters = driveModules[i].getDrivePosition() * DriveConstants.wheelRadiusM;
            modulePositions[i].angle = new Rotation2d(driveModules[i].getRotationPosition());
        }
        Pose2d bootToVehicle = odometry.update(headingRotation, modulePositions);
        ChassisSpeeds measuredChassis = DriveConstants.kinematics.toChassisSpeeds(moduleStates);
        RobotState.getInstance().recordOdometryObservations(bootToVehicle, measuredChassis);

        field2d.setRobotPose(bootToVehicle);
    }

    public void setGoalModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            driveModules[i].setDesiredState(states[i]);
        }
    }

    public void setGoalChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] goalModuleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(goalModuleStates, DriveConstants.maxDriveSpeed);
        setGoalModuleStates(goalModuleStates);
    }

    public Rotation2d getRotation() {
        return new Rotation2d(MathUtil.angleModulus(driveAngle.getHeading()));
    }

    public void resetHeading() {
        driveAngle.resetHeading();
    }

    public double getRoll() {
        return driveAngle.getRoll();
    }

}