/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.frames.Frame;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.AdapterPropagator;
import org.orekit.propagation.events.EclipseDetector;
import org.orekit.propagation.events.handlers.EventHandler;
import org.orekit.propagation.events.handlers.StopOnEvent;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.PVCoordinatesProvider;
import org.orekit.utils.TimeStampedPVCoordinates;

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
    private final AdapterPropagator trajectory;

    /** Indicator for replanned maneuvers. */
    private final boolean replanned;

    /** Eclipse ratio used. This is only used for out-of-plane maneuvers. */
    private double eclipseRatio; 
    
    /** Lost eclipse ratio. This is only used for out-of-plane maneuvers. */
    private double lostEclipseRatio;

	/** DV epsilon. Used to avoid numerical errors when checking if a maneuver is saturated. */
    private final double dV_epsilon = 1e-15;
    
    /** Yaw angle of the maneuver w.r.t its reference direction (rad). This only 
     * applies to mixed longitude-inclination maneuvers. */
    private double yawAngle;
    
    /** Eclipse detector, shared among instances. */
    private static final EclipseDetector eclipseDet;
    
    static {
        try {
            // Without additional parameters, this detector will find umbral 
            // eclipses only. It is TBC that this is the intended result.
            eclipseDet = new EclipseDetector(
                    CelestialBodyFactory.getSun(),  Constants.SUN_RADIUS,
                    CelestialBodyFactory.getEarth(),Constants.WGS84_EARTH_EQUATORIAL_RADIUS);
        } catch (OrekitException ex){
            throw new ExceptionInInitializerError(ex);
        }
    }
    
    /** Simple constructor for maneuvers with zero yaw angle.
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
                             final double thrust, final double isp, final AdapterPropagator trajectory,
                             final boolean replanned) {
        this.model      = model;
        this.date       = date;
        this.deltaV     = deltaV;
        this.thrust     = thrust;
        this.isp        = isp;
        this.trajectory = trajectory;
        this.replanned  = replanned;
        this.eclipseRatio     = -1;
        this.lostEclipseRatio = -1;
        this.yawAngle         = 0.0;
    }
	
	/** Simple constructor.
     * @param model tunable model of the maneuver
     * @param date maneuver date
     * @param deltaV velocity increment in spacecraft frame
     * @param thrust engine thrust
     * @param isp engine specific impulse (s)
     * @param trajectory trajectory to which this maneuver belongs
     * @param replanned if true, the maneuver was missed and has been replanned
	 * @param yawAngle angle of the maneuver (rad)
     */
    public ScheduledManeuver(final TunableManeuver model,
                             final AbsoluteDate date, final Vector3D deltaV,
                             final double thrust, final double isp, final AdapterPropagator trajectory,
                             final boolean replanned, final double yawAngle) {
        this.model      = model;
        this.date       = date;
        this.deltaV     = deltaV;
        this.thrust     = thrust;
        this.isp        = isp;
        this.trajectory = trajectory;
        this.replanned  = replanned;
        this.eclipseRatio     = -1;
        this.lostEclipseRatio = -1;
        this.yawAngle         = yawAngle;
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

    /** Get the signed scalar velocity increment along thrust direction.
    * @return signed scalar velocity increment along thrust direction
    */
    public double getSignedDeltaV() {
        return Vector3D.dotProduct(deltaV, model.getDirection());
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
    public AdapterPropagator getTrajectory() {
        return trajectory;
    }

    /** Get the state a small time before or after maneuver.
     * <p>
     * The time offset should be used to ensure either the
     * state before or after the maneuver is retrieved. If an
     * offset of 0 is used, the result may be either the one
     * before or after due to numerical uncertainties and the
     * discontinuous nature of state around a maneuver. Hence
     * using a dt of 0 is not recommended, but the {@link
     * #getStateBefore()} or {@link #getStateAfter()} methods
     * should be used.
     * </p>
     * @param dt time offset with respect to the maneuver
     * @return state at maneuver time + dt
     * @exception PropagationException if state cannot be propagated
     * to the specified date
     */
    public SpacecraftState getState(final double dt)
        throws PropagationException {
        return trajectory.propagate(date.shiftedBy(dt));
    }

    /** Get the state just before the maneuver.
     * @return state at maneuver time, before maneuver is applied
     * @exception PropagationException if state cannot be propagated
     * to the specified date
     */
    public SpacecraftState getStateBefore()
        throws PropagationException {
        return getState(-0.001).shiftedBy(+0.001);
    }

    /** Get the state just after the maneuver.
     * @return state at maneuver time, after maneuver is applied
     * @exception PropagationException if state cannot be propagated
     * to the specified date
     */
    public SpacecraftState getStateAfter()
        throws PropagationException {
        return getState(+0.001).shiftedBy(-0.001);
    }

    /** Check if the maneuver has been replanned.
     * @return true is the maneuver has been replanned
     */
    public boolean isReplanned() {
        return replanned;
    }

    /** Get the eclipse ratio.
    * @return eclipse ratio
    */
    public double getEclipseRatio() {
        return eclipseRatio;
    }

    /** Get the lost eclipse ratio.
    * @return lost eclipse ratio
    */
    public double getLostEclipseRatio() {
        return lostEclipseRatio;
    }

    /** Update eclipse ratio
     * @param eclipseRatio eclipse ratio
     */
    public void updateEclipseRatio(final double eclipseRatio) {
    	this.eclipseRatio = eclipseRatio;
    }

    /** Update lost eclipse ratio
     * @param lostEclipseRatio lost eclipse ratio
     */
    public void updateLostEclipseRatio(final double lostEclipseRatio) {
    	this.lostEclipseRatio = lostEclipseRatio;
    }

    /** Check if a maneuver is within convergence threshold of another maneuver.
     * @param maneuver other maneuver to check instance against
     * @return true if the two maneuvers share the same model and both their dates
     * and velocity increment are within the model convergence thresholds
     */
    public boolean isWithinThreshold(final ScheduledManeuver maneuver) {        
        return (model.equals(maneuver.model)) &&
               (date.durationFrom(maneuver.date) <= model.getDTConvergence()) &&
               (deltaV.subtract(maneuver.deltaV).getNorm() <= model.getDVConvergence());
    }
    
	/** Check if a maneuver is saturated.
     * @param dV Delta V used to check whether the maneuver is saturated or not
     * @return true if the maneuver is saturated
     */
    public boolean isSaturated(final double dV)
    {
        final boolean isSatInf = dV < 0 && this.getSignedDeltaV() - dV_epsilon > this.getModel().getCurrentDVInf();
        final boolean isSatSup = dV > 0 && this.getSignedDeltaV() + dV_epsilon < this.getModel().getCurrentDVSup();
        return !(isSatInf || isSatSup);
    }

    /** Get the yaw angle of the maneuver.
    * @return yaw angle
    */
    public double getYawAngle() {
        return yawAngle;
    }
    
    /** Returns a boolean indicating whether the maneuver is performed during eclipse or not.
     * It returns true if the spacecraft enters the umbra region of the eclipse during the maneuver
     * @return eclipse flag 
     * @throws OrekitException If the previous state cannot be obtained, or a propagation is required but fails.
     */
    public boolean isManWithinEclipse() throws OrekitException {
        
        // Check first if at the start of the maneuver the spacecraft is already in eclipse
        if (eclipseDet.g(this.getStateBefore()) < 0) {
            return true;
        }
        
        // Atach the eclipse detector to the maneuver trayectory and propagate.
        final double duration = this.getDuration(this.getStateBefore().getMass());
        final AbsoluteDate endDate = this.getDate().shiftedBy(duration);
        final Propagator tmpPropagator = new AdapterPropagator(this.getTrajectory());
        tmpPropagator.addEventDetector(eclipseDet.withHandler(new StopOnEvent<EclipseDetector>()));
        
        final SpacecraftState endStv = tmpPropagator.propagate(this.getDate(), endDate);
        
        // If the propagation stopped before the final date, the S/C switched
        // eclipse state (lighted to/from eclipsed) in the middle.
        return FastMath.abs(endDate.durationFrom(endStv.getDate())) > 1;
    }   
    
}
