/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import java.util.ArrayList;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.optimization.fitting.PolynomialFitter;
import org.apache.commons.math.optimization.general.LevenbergMarquardtOptimizer;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.frames.Transform;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.ManeuverAdapterPropagator;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Station-keeping control attempting to center et parabolique longitude
 * excursion around a specified center target throughout a cycle.
 * <p>
 * This control value is:
 * <pre>
 *   &sum;(l(t)-l<sub>p</sub>(y))<sup>2</sup>
 * </pre>
 * where l(t) is the spacecraft longitude and l<sub>p</sub>(t) is a centered
 * parabolic longitude motion with the same acceleration as the observed motion.
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to 0 attempts to have most of the points longitudes covered by the
 * satellite centered around the l<sub>c</sub> longitude during the
 * station-keeping.
 * </p>
 * @author Luc Maisonobe
 */
public class ParabolicLongitude extends AbstractSKControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Earth model to use to compute longitudes. */
    private final BodyShape earth;

    /** Target longitude. */
    private final double center;

    /** In-plane maneuver model. */
    private final TunableManeuver model;

    /** Maximum number of maneuvers to set up in one cycle. */
    private final int maxManeuvers;

    /** Minimum time between split parts in number of orbits. */
    private final double dtMin;

    /** Reference state at fitting start. */
    private SpacecraftState fitStart;

    /** Iteration number. */
    private int iteration;

    /** Cycle duration. */
    private double cycleDuration;

    /** Date sample during station keeping cycle. */
    private List<Double> dateSample;

    /** Longitude sample during station keeping cycle. */
    private List<Double> longitudeSample;

    /** Fitted dlDot/da. */
    private double dlDotDa;

    /** Fitted initial offset with respect to synchronous semi major axis. */
    private double a0Mas;

    /** Fitted semi-major axis slope. */
    private double aDot;

    /** Fitted initial longitude. */
    private double l0;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model in-plane maneuver model
     * @param maxManeuvers maximum number of maneuvers to set up in one cycle
     * @param orbitsSeparation minimum time between split parts in number of orbits
     * @param lEast longitude slot Eastward boundary
     * @param lWest longitude slot Westward boundary
     * @param samplingStep step to use for sampling throughout propagation
     * @param earth Earth model to use to compute longitudes
     * @exception SkatException if longitudes limits are not sorted properly
     */
    public ParabolicLongitude(final String name,
                              final String controlledName, final int controlledIndex,
                              final TunableManeuver model, final int maxManeuvers,
                              final int orbitsSeparation,
                              final double lEast, final double lWest,
                              final double samplingStep, final BodyShape earth)
        throws SkatException {
        super(name, controlledName, controlledIndex, null, -1,
              FastMath.toDegrees(lWest), FastMath.toDegrees(MathUtils.normalizeAngle(lEast, lWest)));
        if (getMin() >= getMax()) {
            throw new SkatException(SkatMessages.UNSORTED_LONGITUDES,
                                    FastMath.toDegrees(getMin()), FastMath.toDegrees(getMax()));
        }
        this.stephandler     = new Handler();
        this.samplingStep    = samplingStep;
        this.earth           = earth;
        this.center          = 0.5 * (getMin() + getMax());
        this.model           = model;
        this.maxManeuvers    = maxManeuvers;
        this.dtMin           = (orbitsSeparation + 0.5) * Constants.JULIAN_DAY;
        this.dateSample      = new ArrayList<Double>();
        this.longitudeSample = new ArrayList<Double>();
    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException {

        this.iteration     = iteration;
        this.cycleDuration = end.durationFrom(start);

        // gather all special dates (start, end, maneuvers) in one chronologically sorted set
        SortedSet<AbsoluteDate> sortedDates = new TreeSet<AbsoluteDate>();
        sortedDates.add(start);
        sortedDates.add(end);
        AbsoluteDate lastInPlane = start;
        for (final ScheduledManeuver maneuver : maneuvers) {
            final AbsoluteDate date = maneuver.getDate();
            if (maneuver.getName().equals(model.getName()) && date.compareTo(lastInPlane) >= 0) {
                // this is the last in plane maneuver seen so far
                lastInPlane = date;
            }
            sortedDates.add(date);
        }
        for (final ScheduledManeuver maneuver : fixedManeuvers) {
            final AbsoluteDate date = maneuver.getDate();
            if (maneuver.getName().equals(model.getName()) && date.compareTo(lastInPlane) >= 0) {
                // this is the last in plane maneuver seen so far
                lastInPlane = date;
            }
            sortedDates.add(date);
        }

        // select the longest maneuver-free interval after last in plane maneuver for fitting
        AbsoluteDate freeIntervalStart = lastInPlane;
        AbsoluteDate freeIntervalEnd   = lastInPlane;
        AbsoluteDate previousDate      = lastInPlane;
        for (final AbsoluteDate currentDate : sortedDates) {
            if (currentDate.durationFrom(previousDate) > freeIntervalEnd.durationFrom(freeIntervalStart)) {
                freeIntervalStart = previousDate;
                freeIntervalEnd   = currentDate;
            }
            previousDate = currentDate;
        }

        fitStart = propagator.propagate(freeIntervalStart);

        // fit linear model to semi-major axis and quadratic model to mean longitude
        PolynomialFitter aFitter = new PolynomialFitter(1, new LevenbergMarquardtOptimizer());
        PolynomialFitter lFitter = new PolynomialFitter(2, new LevenbergMarquardtOptimizer());
        for (AbsoluteDate date = freeIntervalStart; date.compareTo(freeIntervalEnd) < 0; date = date.shiftedBy(samplingStep)) {
            final SpacecraftState  state = propagator.propagate(date);
            final double meanLongitude = getLongitude(state, PositionAngle.MEAN);
            final double dt = date.durationFrom(freeIntervalStart);
            aFitter.addObservedPoint(dt, state.getA());
            lFitter.addObservedPoint(dt, meanLongitude);
        }

        // polynomial models are:
        // dl(t)/dt  = dlDotDa * [a(t) - as]
        // a(t)      = a(t0) + aDot * (t - t0)
        // l(t)      = l(t0) + dlDotDa * [a(t0) - as] * (t - t0) + dlDotDa/2 * aDot * (t - t0)^2
        final double[] linear = aFitter.fit();
        final double[] quadratic = lFitter.fit();
        l0      = quadratic[0];
        aDot    = linear[1];
        dlDotDa = 2 * quadratic[2] / aDot;
        a0Mas   = quadratic[1] / dlDotDa;

    }

    /** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException {

        // longitude excursion when the cycle is balanced from tPeak - cycleDuration/2
        // to tPeak + cycleDuration/2 where tPeak is the date at which peak longitude
        // is achieved (i.e. dl/dt = 0)
        final double longitudeExcursion = -dlDotDa * aDot * cycleDuration * cycleDuration / 8;
        final double targetPeak         = center + 0.5 * longitudeExcursion;

        final ScheduledManeuver[] tuned;
        final ManeuverAdapterPropagator adapterPropagator = new ManeuverAdapterPropagator(reference);
        if (iteration == 0) {
            // we need to first define the number of maneuvers and their initial settings

            final double deltaA;
            final double lDotDot = dlDotDa * aDot;
            if ((l0 > targetPeak) ^ (lDotDot > 0)) {
                // we are on the wrong side, already "above" target peak (considering curvature direction)

                // TODO
                throw SkatException.createInternalError(null);

            } else {
                // we are on the right side, "below" target peak (considering curvature direction)

                // use the quadratic longitude model to compute the initial semi-major axis offset
                // needed to achieve a peak longitude exactly at target peak
                final double aMas = FastMath.sqrt(2 * aDot * (l0 - targetPeak) / dlDotDa);

                // compute the in plane maneuver required to get this initial semi-major axis offset
                deltaA = aMas - a0Mas;
            }

            final double mu     = fitStart.getMu();
            final double a      = fitStart.getA();
            double totalDeltaV  = FastMath.sqrt(mu * (2 / a - 1 / (a + deltaA))) - FastMath.sqrt(mu / a);

            // fix sign according to thruster direction
            Vector3D thrustDirection =
                    fitStart.getAttitude().getRotation().applyInverseTo(model.getDirection());
            Vector3D velocity = fitStart.getPVCoordinates().getVelocity();
            if (Vector3D.dotProduct(thrustDirection, velocity) < 0) {
                totalDeltaV = -totalDeltaV;
            }

            // compute the number of maneuvers required
            final double limitDV = (totalDeltaV < 0) ? model.getDVInf() : model.getDVSup();
            final int nMan = FastMath.min(maxManeuvers,
                                          (int) FastMath.ceil(FastMath.abs(totalDeltaV / limitDV)));
            final double deltaV = FastMath.max(model.getDVInf(),
                                               FastMath.min(model.getDVSup(), totalDeltaV / nMan));

            tuned = new ScheduledManeuver[tunables.length + nMan];

            // copy existing maneuvers, changing only the trajectory
            for (int i = 0; i < tunables.length; ++i) {
                final ScheduledManeuver maneuver =
                        new ScheduledManeuver(tunables[i].getModel(), tunables[i].getDate(),
                                              tunables[i].getDeltaV(), tunables[i].getThrust(),
                                              tunables[i].getIsp(), adapterPropagator,
                                              tunables[i].isReplanned());
                tuned[i] = maneuver;
                adapterPropagator.addManeuver(maneuver.getDate(), maneuver.getDeltaV(), maneuver.getIsp());
            }

            // add the new maneuvers
            for (int i = 0; i < nMan; ++i) {
                final ScheduledManeuver maneuver =
                        model.buildManeuver(fitStart.getDate().shiftedBy(i * dtMin), deltaV, adapterPropagator);
                tuned[tunables.length + i] = maneuver;
                adapterPropagator.addManeuver(maneuver.getDate(), maneuver.getDeltaV(), maneuver.getIsp());
            }

        } else {

            // adjust the existing maneuvers

            final double lDotDot = dlDotDa * aDot;
            if ((l0 > targetPeak) ^ (lDotDot > 0)) {
                // we are on the wrong side, already "above" target peak (considering curvature direction)

                // TODO
                throw SkatException.createInternalError(null);

            } else {
                // we are on the right side, "below" target peak (considering curvature direction)

                // compute achieved longitude peak
                final double achievedPeak = l0 - dlDotDa * a0Mas * a0Mas / (2 * aDot);

                // compute initial semi major axis offset needed to reach target longitude peak
                final double deltaL = targetPeak - achievedPeak;
                final double deltaSquares = 2 * aDot * deltaL / dlDotDa;
                final double aMas = FastMath.copySign(FastMath.sqrt(a0Mas * a0Mas - deltaSquares), a0Mas);

                // compute the in plane maneuver required to get this initial semi-major axis offset
                final double deltaA = aMas - a0Mas;
                final double mu     = fitStart.getMu();
                final double a      = fitStart.getA();
                double deltaVChange = FastMath.sqrt(mu * (2 / a - 1 / (a + deltaA))) - FastMath.sqrt(mu / a);

                // fix sign according to thruster direction
                Vector3D thrustDirection =
                        fitStart.getAttitude().getRotation().applyInverseTo(model.getDirection());
                Vector3D velocity = fitStart.getPVCoordinates().getVelocity();
                if (Vector3D.dotProduct(thrustDirection, velocity) < 0) {
                    deltaVChange = -deltaVChange;
                }

                // distribute the change over all maneuvers
                int nbMan = 0;
                for (final ScheduledManeuver maneuver : tunables) {
                    if (maneuver.getName().equals(model.getName())) {
                        nbMan++;
                    }
                }

                // apply the changes
                tuned = new ScheduledManeuver[tunables.length];
                for (int i = 0; i < tunables.length; ++i) {
                    final ScheduledManeuver maneuver;
                    if (tunables[i].getName().equals(model.getName())) {
                        // change the maneuver velovity increment
                        double original = Vector3D.dotProduct(tunables[i].getDeltaV(), model.getDirection());
                        maneuver = new ScheduledManeuver(tunables[i].getModel(), tunables[i].getDate(),
                                                         new Vector3D(original + deltaVChange / nbMan,
                                                                      model.getDirection()),
                                                         tunables[i].getThrust(),
                                                         tunables[i].getIsp(), adapterPropagator,
                                                         tunables[i].isReplanned());
                    } else {
                        // copy the maneuver, changing only the trajectory
                        maneuver = new ScheduledManeuver(tunables[i].getModel(), tunables[i].getDate(),
                                                         tunables[i].getDeltaV(), tunables[i].getThrust(),
                                                         tunables[i].getIsp(), adapterPropagator,
                                                         tunables[i].isReplanned());
                    }

                    tuned[i] = maneuver;
                    adapterPropagator.addManeuver(maneuver.getDate(), maneuver.getDeltaV(), maneuver.getIsp());

                }

            }

        }

        return tuned;

    }

    /** Get Earth based longitude.
     * @param state current state
     * @param type type of the angle
     * @return Earth-based longitude
     * @exception OrekitException if frames conversion cannot be computed
     */
    private double getLongitude(final SpacecraftState state, final PositionAngle type)
        throws OrekitException {

        // get equinoctial orbit
        final EquinoctialOrbit orbit = (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state.getOrbit());

        // compute sidereal time
        final Transform transform = earth.getBodyFrame().getTransformTo(state.getFrame(), state.getDate());
        final double theta = transform.transformVector(Vector3D.PLUS_I).getAlpha();

        return MathUtils.normalizeAngle(orbit.getL(type) - theta, center);

    }

    /** {@inheritDoc} */
    public EventDetector getEventDetector() {
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return stephandler;
    }

    /** Inner class for step handling. */
    private class Handler implements OrekitStepHandler {

        /** Serializable UID. */
        private static final long serialVersionUID = 7101089708717693731L;

        /** {@inheritDoc} */
        public void init(final SpacecraftState s0, final AbsoluteDate t) {
            resetMarginsChecks();
            dateSample.clear();
            longitudeSample.clear();
        }

        /** {@inheritDoc} */
        public void handleStep(OrekitStepInterpolator interpolator, boolean isLast)
            throws PropagationException {

            try {

                // find step boundaries
                final AbsoluteDate minDate =
                        interpolator.isForward() ? interpolator.getPreviousDate() : interpolator.getCurrentDate();
                final AbsoluteDate maxDate =
                        interpolator.isForward() ? interpolator.getCurrentDate() : interpolator.getPreviousDate();

                // loop throughout step
                for (AbsoluteDate date = minDate; date.compareTo(maxDate) < 0; date = date.shiftedBy(samplingStep)) {

                    // compute longitudes
                    interpolator.setInterpolatedDate(date);
                    final SpacecraftState state = interpolator.getInterpolatedState();
                    final double trueLongitude = getLongitude(state, PositionAngle.TRUE);
                    final double meanLongitude = getLongitude(state, PositionAngle.MEAN);

                    // check the limits
                    checkMargins(FastMath.toDegrees(trueLongitude));

                    // add point to sample
                    final double dt = date.durationFrom(fitStart.getDate());
                    dateSample.add(dt);
                    longitudeSample.add(meanLongitude);

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
