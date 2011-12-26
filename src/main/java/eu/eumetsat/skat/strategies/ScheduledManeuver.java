/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.orekit.propagation.analytical.ManeuverAdapterPropagator;
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

    /** Tunable maneuver model. */
    private TunableManeuver model;

    /** Maneuver date. */
    private final AbsoluteDate date;

    /** Velocity increment in spacecraft frame. */
    private final Vector3D deltaV;

    /** Engine thrust. */
    private final double thrust;

    /** Specific impulse. */
    private final double isp;

    /** Trajectory to which this maneuver belongs. */
    private final ManeuverAdapterPropagator trajectory;

    /** Indicator for replanned maneuvers. */
    private final boolean replanned;

    /** Simple constructor.
     * @param model tunable model of the maneuver
     * @param date maneuver date
     * @param deltaV velocity increment in spacecraft frame
     * @param thrust engine thrust
     * @param isp engine specific impulse (s)
     * @param trajectory trajectory to which this maneuver belongs
     * @param replanned if true, the maneuver was missed and has been replanned
     */
    public ScheduledManeuver(final TunableManeuver model,
                             final AbsoluteDate date, final Vector3D deltaV,
                             final double thrust, final double isp, final ManeuverAdapterPropagator trajectory,
                             final boolean replanned) {
        this.model      = model;
        this.date       = date;
        this.deltaV     = deltaV;
        this.thrust     = thrust;
        this.isp        = isp;
        this.trajectory = trajectory;
        this.replanned  = replanned;
    }

    /** Get the maneuver name.
     * @return maneuver name
     */
    public String getName() {
        return model.getName();
    }

    /** Get the maneuver model.
     * @return maneuver model
     */
    public TunableManeuver getModel() {
        return model;
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
    public ManeuverAdapterPropagator getTrajectory() {
        return trajectory;
    }

    /** Check if the maneuver has been replanned.
     * @return true is the maneuver has been replanned
     */
    public boolean isReplanned() {
        return replanned;
    }

    /** Check if a maneuver is within convergence threshold of another maneuver.
     * @param maneuver other maneuver to check instance against
     * @return true if the two maneuvers share the same model and both their dates
     * and velocity increment are within the model convergence thresholds
     */
    public boolean isWithinThreshold(final ScheduledManeuver maneuver) {
        return (model == maneuver.model) &&
               (date.durationFrom(maneuver.date) <= model.getDateConvergence()) &&
               (deltaV.subtract(maneuver.deltaV).getNorm() <= model.getDVConvergence());
    }

}
