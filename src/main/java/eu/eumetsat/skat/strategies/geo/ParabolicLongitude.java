/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;
import org.orekit.frames.Frame;
import org.orekit.frames.Transform;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.AdapterPropagator;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.SecularAndHarmonic;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Station-keeping control attempting to center a parabolic longitude
 * excursion around a specified center target throughout a cycle.
 * <p>
 * The longitude drift &zeta; (i.e. time derivative of the mean longitude) depends
 * linearly on semi major axis, and is null at geosynchronous semi major axis:
 * <pre>
 *  dl(t)/dt  = &zeta; = &part;&zeta;/&part;a &times; [a(t) - a<sub>s</sub>]
 * </pre>
 * where a<sub>s</sub> is the synchronous semi major axis and &part;&zeta;/&part;a
 * is a negative coefficient depending only on the gravity field with a theoretical
 * value for Keplerian motion &part;&zeta;/&part;a = -(3/2)&radic;(&mu;/a<sub>s</sub>).
 * </p>
 * <p>
 * The model for semi major axis drift is limited to secular terms only here:
 * <pre>
 * a(t) = a(t<sub>0</sub>) + &aring; &times; (t - t<sub>0</sub>)
 * </pre>
 * where &aring; is the secular time derivative of the semi major axis
 * <p>
 * This implies the model for mean longitude is quadratic:
 * <pre>
 * l(t) = l(t<sub>0</sub>) + &part;&zeta;/&part;a &times; [a(t<sub>0</sub>) - a<sub>s</sub>] &times; (t - t<sub>0</sub>) + &frac12; &part;&zeta;/&part;a &times; &aring; &times; (t - t<sub>0</sub>)<sup>2</sup>
 * </pre>
 * </p>
 * <p>
 * The initial longitude l(t<sub>0</sub>) is inherited from the previous
 * cycle motion. The curvature comes from &aring; which depends on the station-keeping
 * longitude slot. So the only control parameter left is the initial drift, which is
 * adjusted by changing the offset [a(t<sub>0</sub>) - a<sub>s</sub>] at initial time
 * t<sub>0</sub> thanks to in-plane maneuvers. The aim of the control law is to achieve
 * a parabolic motion that is centered in the station-keeping window during one cycle
 * duration, i.e. with a peak (depending on acceleration direction, of cource) longitude
 * at cycle middle time.
 * </p>
 * <p>
 * The model parameters &part;&zeta;/&part;a, a<sub>s</sub> and &aring; are not
 * computed from Keplerian motion, but rather fitted to the real evolution of the
 * parameters against the polynomial models above. The fitting is performed on the
 * longest maneuver-free interval of the cycle after the last in-plane maneuver.
 * </p>
 * <p>
 * The maneuvers aim at putting the end of cycle longitude at a longitude which will
 * start a perfectly centered parabola for next cycle first maneuver. This works
 * regardless of the initial point, which may be either already in the longitude slot,
 * westward to the slot or eastward to the slot. This implies it does work also for
 * longitude changes.
 * </p>
 * <p>
 * If we are too far from the station keeping longitude slot, we set up
 * several maneuvers to reach the slot faster, up to the maximal number of allowed
 * maneuvers par cycle and fulfilling the constraints on velocity increment sizes.
 * </p>
 * @author Luc Maisonobe
 */
public class ParabolicLongitude extends AbstractSKControl {

    /** Earth pulsation, one day period. */
    private static final double EARTH_PULSATION = 2.0 * FastMath.PI / Constants.JULIAN_DAY;

    /** Moon pulsation (synodic period). */
    private static final double MOON_PULSATION = 2.0 * FastMath.PI / (29.530589 * Constants.JULIAN_DAY);

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Earth model to use to compute longitudes. */
    private final BodyShape earth;

    /** Target longitude. */
    private final double center;

    /** Maximum number of maneuvers to set up in one cycle. */
    private final int maxManeuvers;

    /** Time offset of the first maneuver with respect to cycle start. */
    private final double firstOffset;

    /** Minimum time between split parts in number of orbits. */
    private final int orbitsSeparation;

    /** Reference state at fitting start. */
    private SpacecraftState fitStart;

    /** Iteration number. */
    private int iteration;

    /** Cycle start. */
    private AbsoluteDate cycleStart;

    /** Cycle end. */
    private AbsoluteDate cycleEnd;

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

    /** Fitting coefficients for a. */
    private double[] fittedA;

    /** Fitting coefficients for longitude. */
    private double[] fittedL;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model in-plane maneuver model
     * @param yawFlipSequence array of pairs containing a day-of-year and the corresponding new status of the yaw flip (0 or 1)
     * @param firstOffset time offset of the first maneuver with respect to cycle start
     * @param maxManeuvers maximum number of maneuvers to set up in one cycle
     * @param orbitsSeparation minimum time between split parts in number of orbits
     * @param lEast longitude slot Eastward boundary
     * @param lWest longitude slot Westward boundary
     * @param samplingStep step to use for sampling throughout propagation
     * @param horizon time horizon duration
     * @param earth Earth model to use to compute longitudes
     * @exception SkatException if longitudes limits are not sorted properly
     */
    public ParabolicLongitude(final String name,
                              final String controlledName, final int controlledIndex,
                              final TunableManeuver[] model, int[][] yawFlipSequence, final double firstOffset,
                              final int maxManeuvers, final int orbitsSeparation,
                              final double lEast, final double lWest,
                              final double samplingStep, final double horizon,
                              final BodyShape earth)
        throws SkatException {
        super(name, model, yawFlipSequence, controlledName, controlledIndex, null, -1,
              FastMath.toDegrees(lWest), FastMath.toDegrees(MathUtils.normalizeAngle(lEast, lWest)),
              horizon * Constants.JULIAN_DAY);
        if (getMin() >= getMax()) {
            throw new SkatException(SkatMessages.UNSORTED_LONGITUDES,
                                    FastMath.toDegrees(getMin()), FastMath.toDegrees(getMax()));
        }
        this.stephandler      = new Handler();
        this.samplingStep     = samplingStep;
        this.earth            = earth;
        this.center           = 0.5 * (lWest + MathUtils.normalizeAngle(lEast, lWest));
        this.firstOffset      = firstOffset;
        this.maxManeuvers     = maxManeuvers;
        this.orbitsSeparation = orbitsSeparation;
        this.dateSample       = new ArrayList<Double>();
        this.longitudeSample  = new ArrayList<Double>();

        // rough order of magnitude for initialization purposes only
        this.fittedA          = new double[] {
            42165000.0, 0.0, 100.0, 100.0, 1000.0, 1000.0
        };
        this.fittedL          = new double[] {
            center, 0.0, 0.0, 1.0e-4, 1.0e-4, 1.0e-4, 1.0e-4
        };

    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate cycleStart, final AbsoluteDate timeHorizonEnd)
        throws OrekitException {

        this.iteration  = iteration;
        this.cycleStart = cycleStart;
        this.cycleEnd   = cycleStart.shiftedBy(getCycleDuration());
        resetMarginsChecks();

        // select a long maneuver-free interval for fitting
        final AbsoluteDate[] freeInterval = getManeuverFreeInterval(maneuvers, fixedManeuvers, cycleStart, timeHorizonEnd);

        if (iteration == 0) {
            clearHistory();
            final double delta = orbitsSeparation * propagator.propagate(freeInterval[0]).getKeplerianPeriod();
            final AbsoluteDate date1 = freeInterval[0].shiftedBy(delta);
            final AbsoluteDate date2 = cycleStart.shiftedBy(firstOffset);
            AbsoluteDate fitStartDate = (date1.compareTo(date2) > 0) ? date1 : date2;
            if (fitStartDate.compareTo(freeInterval[1]) > 0) {
                fitStartDate  = freeInterval[1];
            }
            fitStart = propagator.propagate(fitStartDate);
        } else {
            fitStart = propagator.propagate(fitStart.getDate());
        }

        // fit linear model to semi-major axis and quadratic model to mean longitude
        SecularAndHarmonic aModel = new SecularAndHarmonic(1,
                                                           new double[] {
                                                               2 * MOON_PULSATION,
                                                               2 * (EARTH_PULSATION - MOON_PULSATION)
                                                           });
        SecularAndHarmonic lModel = new SecularAndHarmonic(2,
                                                           new double[] {
                                                               2 * MOON_PULSATION,
                                                               2 * (EARTH_PULSATION - MOON_PULSATION)
                                                           });
        aModel.resetFitting(fitStart.getDate(), fittedA);
        lModel.resetFitting(fitStart.getDate(), fittedL);
        for (AbsoluteDate date = freeInterval[0]; date.compareTo(freeInterval[1]) < 0; date = date.shiftedBy(samplingStep)) {
            final SpacecraftState  state = propagator.propagate(date);
            final double meanLongitude = getMeanLongitude(state);
            aModel.addPoint(date, state.getA());
            lModel.addPoint(date, meanLongitude);
        }

        // polynomial models are:
        // dl(t)/dt  = dlDotDa * [a(t) - as]
        // a(t)      = a(t0) + aDot * (t - t0)
        // l(t)      = l(t0) + dlDotDa * [a(t0) - as] * (t - t0) + dlDotDa/2 * aDot * (t - t0)^2
        aModel.fit();
        lModel.fit();
        fittedA = aModel.getFittedParameters();
        final double[] meanA = aModel.approximateAsPolynomialOnly(1, fitStart.getDate(), 1, 0,
                                                                  freeInterval[0], freeInterval[1], samplingStep);
        fittedL = lModel.getFittedParameters();
        final double[] meanL = lModel.approximateAsPolynomialOnly(2, fitStart.getDate(), 2, 0,
                                                                  freeInterval[0], freeInterval[1], samplingStep);
        l0      = meanL[0];
        aDot    = meanA[1];
        dlDotDa = 2 * meanL[2] / aDot;
        a0Mas   = meanL[1] / dlDotDa;

        addQuadraticFit(meanL, 0, freeInterval[1].durationFrom(fitStart.getDate()));

    }

    /** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException {

        if (loopDetected()) {
            // we are stuck in a convergence loop, we cannot improve the current solution
            return tunables;
        }
        
        final ScheduledManeuver[] tuned;
        final AdapterPropagator adapterPropagator = new AdapterPropagator(reference);
        if (iteration == 0) {
            // we need to first define the number of maneuvers and their initial settings
            final double totalDeltaV  = computeRequiredDV();

            // compute the number of maneuvers required
            final TunableManeuver model = getModel();
            final double limitDV = (totalDeltaV < 0) ? model.getCurrentDVInf() : model.getCurrentDVSup();
            final int    nMan    = FastMath.min(maxManeuvers,
                                                (int) FastMath.ceil(FastMath.abs(totalDeltaV / limitDV)));
            final double deltaV  = FastMath.max(model.getCurrentDVInf(),
                                                FastMath.min(model.getCurrentDVSup(), totalDeltaV / nMan));

            tuned = new ScheduledManeuver[tunables.length + nMan];
            System.arraycopy(tunables, 0, tuned, 0, tunables.length);
            changeTrajectory(tuned, 0, tunables.length, adapterPropagator);

            // in order to avoid tempering eccentricity too much,
            // we use a (n+1/2) orbits between maneuvers, where n is an integer
            final double separation = (orbitsSeparation + 0.5) * fitStart.getKeplerianPeriod();

            // add the new maneuvers
            for (int i = 0; i < nMan; ++i) {
                tuned[tunables.length + i] =
                        new ScheduledManeuver(model, fitStart.getDate().shiftedBy(i * separation),
                                              new Vector3D(deltaV, model.getDirection()),
                                              model.getCurrentThrust(), model.getCurrentISP(),
                                              adapterPropagator, false);
            }

        } else {

            // adjust the existing maneuvers
            double deltaVChange = computeDVChange();

            // distribute the change over all maneuvers
            tuned = tunables.clone();
            changeTrajectory(tuned, 0, tuned.length, adapterPropagator);
            distributeDV(deltaVChange, 0.0, tuned, adapterPropagator);

        }

        // finalize propagator
        for (final ScheduledManeuver maneuver : tuned) {
            adapterPropagator.addEffect(new SmallManeuverAnalyticalModel(maneuver.getStateBefore(),
                                                                         maneuver.getDeltaV(),
                                                                         maneuver.getIsp()));
        }

        return tuned;

    }

    /** Get Earth based mean longitude.
     * @param state current state
     * @return Earth based mean longitude
     * @exception OrekitException if frames conversion cannot be computed
     */
    private double getMeanLongitude(final SpacecraftState state)
        throws OrekitException {

        // get equinoctial orbit
        final EquinoctialOrbit orbit = (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state.getOrbit());

        // compute sidereal time
        final Transform transform = earth.getBodyFrame().getTransformTo(state.getFrame(), state.getDate());
        final double theta = transform.transformVector(Vector3D.PLUS_I).getAlpha();

        return MathUtils.normalizeAngle(orbit.getL(PositionAngle.MEAN) - theta, center);

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
                    final Frame earthFrame      = earth.getBodyFrame();
                    final double geogLongitude  = state.getPVCoordinates(earthFrame).getPosition().getAlpha();
                    final double meanLongitude  = getMeanLongitude(state);

                    // Check limits only during the current cycle
                    if (date.durationFrom(cycleStart) <= ParabolicLongitude.this.getCycleDuration() ) {
                        checkMargins(date, FastMath.toDegrees(geogLongitude));
                    }

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
    
    /** Compute the initial guess of the Delta V required to perform the maneuver.
     * @return Delta V required to perform the maneuver
     * @throws OrekitException
     */
    public double computeRequiredDV() throws OrekitException
    {
        // Compute time until the end of the cycle and longitude at the end of the cycle
        final double lEnd    = computeLEnd();
        final double tEndMt0 = computeTEndMt0();

        // use the quadratic longitude model to compute the initial semi-major axis offset
        // needed to achieve a longitude at next cycle first maneuver which will be exactly
        // at the start of a centered parabola
        final double aMas   = (lEnd - l0) / (dlDotDa * tEndMt0) - 0.5 * aDot * tEndMt0;

        // compute the in plane maneuver required to get this initial semi-major axis offset
        final double deltaA       = aMas - a0Mas;
        final double mu           = fitStart.getMu();
        final double a            = fitStart.getA();
        final double totalDeltaV  = thrustSignVelocity(fitStart,getModel()) *
                                    FastMath.sqrt(mu * (2 / a - 1 / (a + deltaA))) - FastMath.sqrt(mu / a);
        return totalDeltaV;        
    }
    
    /** Compute the change on the Delta V of the maneuver to achieve the target 
     * at the end of the cycle longitude.
     * @return Change on Delta V
     * @throws OrekitException
     */
    public double computeDVChange() throws OrekitException
    {
        // Compute time until the end of the cycle and longitude at the end of the cycle
        final double lEnd    = computeLEnd();
        final double tEndMt0 = computeTEndMt0();
        
        // compute the achieved longitude at cycle end
        final double achievedEnd = l0 + dlDotDa * (a0Mas + 0.5 * aDot * tEndMt0) * tEndMt0;

        // compute initial semi major axis offset needed to reach target longitude end
        final double deltaL = lEnd - achievedEnd;
        final double deltaA = deltaL / (dlDotDa * tEndMt0);


        // compute the in plane maneuver required to get this initial semi-major axis offset
        final double mu           = fitStart.getMu();
        final double a            = fitStart.getA();
        double deltaVChange = thrustSignVelocity(fitStart,getModel()) *
                                    FastMath.sqrt(mu * (2 / a - 1 / (a + deltaA))) - FastMath.sqrt(mu / a);
        return deltaVChange;
    }
    
    /** Compute the target longitude at the end of the cycle.
     * @return longitude at the end of the cycle
     */
    private double computeLEnd()
    {
        // longitude excursion when the cycle is centered from tPeak - cycleDuration/2
        // to tPeak + cycleDuration/2 where tPeak is the date at which peak longitude
        // is achieved (i.e. dl/dt = 0)
        final double cycleDuration      = super.getCycleDuration();
        final double longitudeExcursion = -dlDotDa * aDot * cycleDuration * cycleDuration / 8;

        // we want to achieve a longitude at next cycle first maneuver which will be
        // exactly at the start of a centered parabola
        return center - 0.5 * longitudeExcursion;       
    }
    
    /** Compute the time elapsed until the end of the cycle.
     * @return time until the end of the cycle
     */
    private double computeTEndMt0()
    {
        return cycleEnd.durationFrom(fitStart.getDate()) + firstOffset;
        // tEndMt0 = cycleDuration;
    }

}
