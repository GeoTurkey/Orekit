/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.orekit.propagation.Propagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;

/**
 * This class is a simple container for impulse maneuver that are completely defined.
 * <p>
 * Instances of this class are guaranteed to be immutable.
 * </p>
 * @author Luc Maisonobe
 */
public class ScheduledManeuver {

    /** Maneuver name. */
    private String name;

    /** Indicator for in-plane maneuvers. */
    private final boolean inPlane;

    /** Maneuver date. */
    private final AbsoluteDate date;

    /** Velocity increment in spacecraft frame. */
    private final Vector3D deltaV;

    /** Engine thrust. */
    private final double thrust;

    /** Specific impulse. */
    private final double isp;

    /** Trajectory to which this maneuver belongs. */
    private final Propagator trajectory;

    /** Simple constructor.
     * @param inPlane if true, the maneuver is considered to be in-plane
     * @param date maneuver date
     * @param deltaV velocity increment in spacecraft frame
     * @param thrust engine thrust
     * @param isp engine specific impulse (s)
     * @param trajectory trajectory to which this maneuver belongs
     */
    public ScheduledManeuver(final String name, final boolean inPlane, final AbsoluteDate date,
                             final Vector3D deltaV, final double thrust, final double isp,
                             final Propagator trajectory) {
        this.name       = name;
        this.inPlane    = inPlane;
        this.date       = date;
        this.deltaV     = deltaV;
        this.thrust     = thrust;
        this.isp        = isp;
        this.trajectory = trajectory;
    }

    /** Get the maneuver name.
     * @return maneuver name
     */
    public String getName() {
        return name;
    }

    /** Check if the maneuver is in-plane.
     * @return true is the maneuver is in-plane
     */
    public boolean isInPlane() {
        return inPlane;
    }

    /** Get the maneuver date.
    * @return maneuver date
    */
    public AbsoluteDate getDate() {
        return date;
    }

    /** Get the velocity increment in spacecraft frame.
    * @return velocity increment in spacecraft frame
    */
    public Vector3D getDeltaV() {
        return deltaV;
    }

    /** Get the engine thrust.
    * @return engine thrust
    */
    public double getThrust() {
        return thrust;
    }

    /** Get the specific impulse.
    * @return specific impulse
    */
    public double getIsp() {
        return isp;
    }

    /** Get the maneuver duration.
     * @param mass spacecraft mass at burn start
     * @return maneuver duration in seconds
     */
    public double getDuration(final double mass) {
        final double exhaustVelocity = Constants.G0_STANDARD_GRAVITY * isp;
        final double flowRate        = thrust / exhaustVelocity;
        final double consumption     = -mass * FastMath.expm1(-deltaV.getNorm() / exhaustVelocity);
        return consumption / flowRate;
    }

    /** Get the trajectory to which this maneuver belongs.
     * @return trajectory
     */
    public Propagator getTrajectory() {
        return trajectory;
    }

}
