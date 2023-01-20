package frc.robot.util;

import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Stores a history of timestamped Pose2d objects.
 */

public class PoseHistory {

    /**
     * PoseHistory Class Contains
     *      Interpolating Timestamp Class
     *      Timestamped Pose2d Class
     */


    //Inner class handling interpolation of double values
    
    /**
     * Interpolation: estimate the values of new data points based 
     * on a range of known data points;
     * 
     * Inter (Inside) means the new data points fall within the range of known points
     */

    /**
     * Comparable interface places data held in the InterpolatingTimestamp object in a particular
     * order so they can be compared.  
     */

    private static class InterpolatingTimestamp implements Comparable<InterpolatingTimestamp> {
        public double value;

        public InterpolatingTimestamp(double value) {
            this.value = value;
        }

        /**
         * dydx is set to total value change between current and other InterpolatingTimestamp Objects
         *      In this way dydx acts as the derivative or slope between the two 
         *    
         * where x (which is between 0 and 1) is a percent of the change between the original two data points 
         *      so dydx * x = data point 50% of the way between current and other InterpolatingTimestamp Objects
         * 
         * This is added to the value of original InterpolatingTimestamp
         * 
         * So InterpolatingTimestamp(SearchY), the predicted new data point based on the old, is returned  
         */

        public InterpolatingTimestamp interpolate(InterpolatingTimestamp other, double x) {
            double dydx = other.value - value;
            double searchY = dydx * x + value;
            return new InterpolatingTimestamp(searchY);
        }


        /**
         * @param upper timestamp at end of known range 
         * @param query given timestamp with unknown value (in our case pose) associated with its key 
         * @return scale factor of range current to upper that will produce timestamp of query 
         */

        public double inverseInterpolate(InterpolatingTimestamp upper, InterpolatingTimestamp query) {

            //if distance between either upper to current value or query to current value is less than zero, return 0
            double upper_to_lower = upper.value - value;
            if (upper_to_lower <= 0) {
                return 0;
            }
            double query_to_lower = query.value - value;
            if (query_to_lower <= 0) {
                return 0;
            }
            return query_to_lower / upper_to_lower;
        }

        @Override

        //Compares values held within two InterpolatingTimestamp Objects 
        public int compareTo(InterpolatingTimestamp other) {
            return Double.compare(value, other.value);
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            InterpolatingTimestamp that = (InterpolatingTimestamp) o;
            return Double.compare(that.value, value) == 0;
        }

        @Override
        public int hashCode() {
            return Objects.hash(value);
        }
    }

    public static class TimestampedPose2d {
        private final InterpolatingTimestamp timestamp;
        private final Pose2d pose;

        private TimestampedPose2d(InterpolatingTimestamp timestamp, Pose2d pose) {
            this.timestamp = timestamp;
            this.pose = pose;
        }

        /**
         * @return The timestamp that the pose was recorded, in seconds
         */
        public double getTimestamp() {
            return timestamp.value;
        }

        /**
         * @return The pose associated with the timestamp
         */
        public Pose2d getPose() {
            return pose;
        }
    }

    private final int capacity;

    /**
     * Declares TreeMap object pairs a timestamp with a pose2d of the robot
     * 
     * Treemap which implements Map is a java import 
     * 
     * Generic Types:
     *  K-key   In our case timestamps
     *  V-value     In our case poses 
     */
    private final TreeMap<InterpolatingTimestamp, Pose2d> map = new TreeMap<>();

    /**
     * Creates a new PoseHistory with the given capacity.  When the history is at capacity, the oldest poses are removed
     * as new ones are inserted. 
     * 
     * This way, PoseHistory will keep a constant log with length capacity of previous values
     * 
     * @param capacity The capacity of the history
     */

    //PoseHistory Class Constructor 
    public PoseHistory(int capacity) {
        this.capacity = capacity;
    }

    /**
     * Creates a new PoseHistory with infinite capacity. This is usually not a good idea in practice, because it can
     * lead to high memory usage potentially without the user knowing
     */
    public PoseHistory() {
        this(0);
    }

    /**
     * Resets the pose history, deleting all entries.
     */
    public void reset() {
        map.clear();
    }

    /**
     * Inserts a new timestamped pose into the history.
     * @param timestamp The timestamp, in seconds
     * @param pose The pose
     */
    public void insert(double timestamp, Pose2d pose) {

        //while the size exceeds capacity 
        while (capacity > 0 && map.size() >= capacity) {
            //Remove elements since the tree is oversize
            map.remove(map.firstKey());
        }

        //Adds a new K,V pair to the TreeMap 
        map.put(new InterpolatingTimestamp(timestamp), pose);
    }

    /**
     * Gets the latest timestamp and pose from the history.
     * @return An object containing the timestamp and the pose
     */

    /**
     * Optional is a container; has methods to make it easy to determine if it holds a null value 
     * 
     * lastEntry is a java imported method for Maps
     */
    public Optional<TimestampedPose2d> getLatest() {
        Map.Entry<InterpolatingTimestamp, Pose2d> entry = map.lastEntry();
        if (entry == null) {
            return Optional.empty();
        }

        /**
         * Returns Optional containing TimestampedPose2d Object 
         * K & V (Timestamp and Pose) of latest entry in the map 
         */

        return Optional.of(new TimestampedPose2d(entry.getKey(), entry.getValue()));
    }

    /**
     * Interpolates between two poses based on the scale factor t.  For example, t=0 would result in the first pose,
     * t=1 would result in the last pose, and t=0.5 would result in a pose which is exactly halfway between the two
     * poses.  Values of t less than zero return the first pose, and values of t greater than 1 return the last pose.
     * @param lhs The left hand side, or first pose to use for interpolation
     * @param rhs The right hand side, or last pose to use for interpolation
     * @param t The scale factor, 0 <= t <= 1
     * @return The pose which represents the interpolation.  For t <= 0, the "lhs" parameter is returned directly.
     *         For t >= 1, the "rhs" parameter is returned directly.
     */
    private static Pose2d interpolate(Pose2d lhs, Pose2d rhs, double t) {

        /**
         * Interpolation can only estimate within the range of known data, so if the scale factor
         * requests extrapolated data (in this case t<=0 or t>=1) it returns the farthest known value
         * in that direction
         */

        if (t <= 0) {
            return lhs;
        } else if (t >= 1) {
            return rhs;
        }

        /**
         * Twist2d represents change in distance along an arc
         * 
         * dx - Change in x direction relative to robot.
         * dy - Change in y direction relative to robot.
         * dtheta - Change in angle relative to robot.
         */

        //sets twist to a Twist2d that maps lhs (start) to rhs (end)
        Twist2d twist = lhs.log(rhs);

        /**
         * Scaled is set to a Twist2d that is at a certain point between lhs and rhs based on t 
         * twist is essentially treated as the derivative or slope over the range 
         */
        Twist2d scaled = new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);

        /**
         * exp returns a pose generated by lhs (starting pose) and position change (Twist2d scaled)
         * so this is the estimated pose based on interpolation 
         */
        return lhs.exp(scaled);
    }

    /**
     * Retrieves a pose at the given timestamp.  If no pose is available at the requested timestamp, interpolation is
     * performed between the two timestamps nearest to the one requested.
     * @param timestamp The timestamp to obtain a pose at
     * @return An Optional object which potentially contains the located pose, or is empty if no pose could be computed.
     */
    public Optional<Pose2d> get(double timestamp) {

        InterpolatingTimestamp key = new InterpolatingTimestamp(timestamp);
        Pose2d retrieved = map.get(key);

        if (retrieved != null) return Optional.of(retrieved); //We have a pose at the exact timestamp, return it

        //find which keys "sandwich" the given key 
        InterpolatingTimestamp topBound = map.ceilingKey(key);
        InterpolatingTimestamp bottomBound = map.floorKey(key);

        //If attempting interpolation at ends of tree, return the nearest data point
        if (topBound == null && bottomBound == null) {
            return Optional.empty();
        } else if (topBound == null) { //If there is no top "sandwich" the nearest will be the bottom "sandwich"
            return Optional.of(map.get(bottomBound));
        } else if (bottomBound == null) { //If there is no bottom "sandwich" the nearest will be the top "sandwich"
            return Optional.of(map.get(topBound));
        }

        //Get surrounding values for interpolation
        //Basically if neither topBound or bottomBound is null, interpolate b
        Pose2d topElem = map.get(topBound);
        Pose2d bottomElem = map.get(bottomBound);

        /**
         * returns TimestampPose2d based on interpolation between the "sandwich" (topElem and bottomElem)
         * 
         * inverseInterpolate returns scale factor that will produce the value associated with 
         * timestamp of key (which is the desired information)
         */
        return Optional.of(interpolate(bottomElem, topElem, bottomBound.inverseInterpolate(topBound, key)));
    }
}