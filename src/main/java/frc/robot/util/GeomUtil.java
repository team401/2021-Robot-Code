package frc.robot.util;


import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

/**
 * Geometry utilities for working with translations, rotations, transforms, and poses.  Provides the following features:
 * * Singleton identity instances for all four geometry classes
 * * Methods for creating poses and transforms from a pure rotation or a pure translation
 * * Methods for converting Pose2d objects to Transform2d objects and vice versa
 * * Methods for converting translations, transforms, and poses between meters and inches
 */
public class GeomUtil {
    /**
     * A Translation2d which represents the identity (zero) translation (x=0, y=0)
     * Applying the identity translation to a translation or a pose will result in the same translation or pose.
     */
    public static final Translation2d TRANSLATION_ZERO = new Translation2d();

    /**
     * A Rotation2d which represents the identity (zero) rotation (theta=0)
     * Applying the identity rotation to a translation, rotation, or pose will result in the same translation, rotation, or pose.
     */
    public static final Rotation2d ROTATION_ZERO = new Rotation2d();

    /**
     * A Pose2d which represents the identity (origin) pose (x=0, y=0, theta=0)
     */
    public static final Pose2d POSE_ZERO = new Pose2d(TRANSLATION_ZERO, ROTATION_ZERO);

    /**
     * A Transform2d which represents the identity (zero) transform (x=0, y=0, theta=0)
     * Applying the identity transform to a pose will result in the same pose.
     */
    public static final Transform2d TRANSFORM_ZERO = new Transform2d(TRANSLATION_ZERO, ROTATION_ZERO);

    /**
     * A Twist2d which represents the identity (zero) twist (dx=0, dy=0, dtheta=0)
     */
    public static final Twist2d TWIST_ZERO = new Twist2d();

    /**
     * Inverts a pose.  This is the same as inverting a Transform2d, but does not require converting to that.
     * Useful for inverting the start of a kinematic chain
     * @param pose The pose to invert
     * @return The inverted pose
     */
    public static Pose2d poseInverse(Pose2d pose) {
        Rotation2d rotationInverted = pose.getRotation().unaryMinus();
        return new Pose2d(pose.getTranslation().unaryMinus().rotateBy(rotationInverted), rotationInverted);
    }

    /**
     * Creates a pure translating transform
     * @param translation The translation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d transformFromTranslation(Translation2d translation) {
        return new Transform2d(translation, ROTATION_ZERO);
    }

    public static Transform2d transformFromTranslation(double x, double y) {
        return new Transform2d(new Translation2d(x, y), ROTATION_ZERO);
    }

    /**
     * Creates a pure rotating transform
     * @param rotation The rotation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d transformFromRotation(Rotation2d rotation) {
        return new Transform2d(TRANSLATION_ZERO, rotation);
    }

    /**
     * Creates a pure translated pose
     * @param translation The translation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d poseFromTranslation(Translation2d translation) {
        return new Pose2d(translation, ROTATION_ZERO);
    }

    /**
     * Creates a pure rotated pose
     * @param rotation The rotation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d poseFromRotation(Rotation2d rotation) {
        return new Pose2d(TRANSLATION_ZERO, rotation);
    }

    /**
     * Converts a Pose2d to a Transform2d to be used in a kinematic chain
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform2d poseToTransform(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic chain
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose2d transformToPose(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Converts a Translation2d measured in inches to a Translation2d measured in meters
     * @param inches A Translation2d measured in inches
     * @return The equivalent Translation2d measured in meters
     */
    public static Translation2d inchesToMeters(Translation2d inches) {
        return new Translation2d(
                Units.inchesToMeters(inches.getX()),
                Units.inchesToMeters(inches.getY())
        );
    }

    /**
     * Converts a Translation2d measured in meters to a Translation2d measured in inches
     * @param meters A Translation2d measured in meters
     * @return The equivalent Translation2d measured in inches
     */
    public static Translation2d metersToInches(Translation2d meters) {
        return new Translation2d(
                Units.metersToInches(meters.getX()),
                Units.metersToInches(meters.getY())
        );
    }

    /**
     * Converts a Transform2d measured in inches to a Transform2d measured in meters
     * @param inches A Transform2d measured in inches
     * @return The equivalent Transform2d measured in meters
     */
    public static Transform2d inchesToMeters(Transform2d inches) {
        return new Transform2d(inchesToMeters(inches.getTranslation()), inches.getRotation());
    }

    /**
     * Converts a Transform2d measured in meters to a Transform2d measured in inches
     * @param meters A Transform2d measured in meters
     * @return The equivalent Transform2d measured in inches
     */
    public static Transform2d metersToInches(Transform2d meters) {
        return new Transform2d(metersToInches(meters.getTranslation()), meters.getRotation());
    }

    /**
     * Converts a Pose2d measured in inches to a Pose2d measured in meters
     * @param inches A Pose2d measured in inches
     * @return The equivalent Pose2d measured in meters
     */
    public static Pose2d inchesToMeters(Pose2d inches) {
        return new Pose2d(inchesToMeters(inches.getTranslation()), inches.getRotation());
    }

    /**
     * Converts a Pose2d measured in meters to a Pose2d measured in inches
     * @param meters A Pose2d measured in meters
     * @return The equivalent Pose2d measured in inches
     */
    public static Pose2d metersToInches(Pose2d meters) {
        return new Pose2d(metersToInches(meters.getTranslation()), meters.getRotation());
    }

    /**
     * Interpolates between two poses based on the scale factor t.  For example, t=0 would result in the first pose,
     * t=1 would result in the last pose, and t=0.5 would result in a pose which is exactly halfway between the two
     * poses.  Values of t less than zero return the first pose, and values of t greater than 1 return the last pose.
     * @param lhs The left hand side, or first pose to use for interpolation
     * @param rhs The right hand side, or last pose to use for interpolation
     * @param t The scale factor, between 0 and 1, inclusive
     * @return The pose which represents the interpolation.  For t less than 0, the "lhs" parameter is returned directly.
     *         For t greater than or equal to 1, the "rhs" parameter is returned directly.
     */
    public static Pose2d interpolate(Pose2d lhs, Pose2d rhs, double t) {
        if (t <= 0) {
            return lhs;
        } else if (t >= 1) {
            return rhs;
        }
        Twist2d twist = lhs.log(rhs);
        Twist2d scaled = new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
        return lhs.exp(scaled);
    }

    /**
     * Returns the direction that this translation makes with the origin as a Rotation2d
     * @param translation The translation
     * @return The direction of the translation
     */
    public static Rotation2d direction(Translation2d translation) {
        return new Rotation2d(translation.getX(), translation.getY());
    }
}