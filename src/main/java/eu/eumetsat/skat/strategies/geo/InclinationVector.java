/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import java.util.List;

import org.apache.commons.math.analysis.ParametricUnivariateFunction;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.optimization.fitting.CurveFitter;
import org.apache.commons.math.optimization.general.LevenbergMarquardtOptimizer;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.OrbitType;
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

/**
 * Station-keeping control for inclination vector.
 * <p>
 * The mean inclination vector moves along a line mainly due to third
 * body attraction with 13 days and 6 months long period variations
 * (half Moon period and half Sun period), and also some short period
 * terms.
 * </p>
 * <p>
 * This control law attempts to control only the secular motion and
 * bring it back to a reference point only when it escapes an allowed
 * circle.
 * </p>
 * <p>
 * The control law simply fits the actual motion with a simplified
 * model with secular and long period terms and when this motion escapes
 * the allowed region, it tries to bring the secular part back to the
 * prescribed point.
 * </p>
 * <p>
 * This control law tunes maneuvers by using an out-of-plane maneuver,
 * which may be split if it exceeds maximal &Delta;V.
 * </p>
 * @author Luc Maisonobe
 */
public class InclinationVector extends AbstractSKControl {

    /** Sun pulsation, 6 months period. */
    private static final double SUN_PULSATION = 4.0 * FastMath.PI / Constants.JULIAN_YEAR;

    /** Moon pulsation (two weeks period, we use synodic period here). */
    private static final double MOON_PULSATION = 4.0 * FastMath.PI / (29.530589 * Constants.JULIAN_DAY);

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Maximum number of maneuvers to set up in one cycle. */
    private final int maxManeuvers;

    /** Time offset of the first maneuver with respect to cycle start. */
    private final double firstOffset;

    /** Minimum time between split parts in number of orbits. */
    private final int orbitsSeparation;

    /** Iteration number. */
    private int iteration;

    /** Abscissa of reference inclination vector. */
    private final double referenceHx;

    /** Ordinate of target inclination vector. */
    private final double referenceHy;

    /** Number of maneuvers to perform. */
    private int nbMan;

    /** Abscissa of start inclination vector before maneuver. */
    private double startHx;

    /** Ordinate of start inclination vector before maneuver. */
    private double startHy;

    /** Abscissa of target inclination vector. */
    private double targetHx;

    /** Ordinate of target inclination vector. */
    private double targetHy;

    /** Radius of the inner circle (internally osculating the limit inclination angle circle). */
    private final double innerRadius;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Reference state at fitting start. */
    private SpacecraftState fitStart;

    /** Parameters of the fitted inclination model. */
    private double[][] fitted;

    /** Cycle start. */
    private AbsoluteDate cycleStart;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model out-of-plane maneuver model
     * @param firstOffset time offset of the first maneuver with respect to cycle start
     * @param maxManeuvers maximum number of maneuvers to set up in one cycle
     * @param orbitsSeparation minimum time between split parts in number of orbits
     * @param referenceHx abscissa of reference inclination vector
     * @param referenceHy ordinate of reference inclination vector
     * @param limitInclination limit circle radius
     * @param samplingStep step to use for sampling throughout propagation
     */
    public InclinationVector(final String name, final String controlledName, final int controlledIndex,
                             final TunableManeuver model, final double firstOffset,
                             final int maxManeuvers, final int orbitsSeparation,
                             final double referenceHx, final double referenceHy,
                             final double limitInclination, final double samplingStep) {

        super(name, model, controlledName, controlledIndex, null, -1, 0.0, FastMath.toDegrees(limitInclination));

        this.stephandler      = new Handler();
        this.firstOffset      = firstOffset;
        this.maxManeuvers     = maxManeuvers;
        this.orbitsSeparation = orbitsSeparation;
        this.referenceHx      = referenceHx;
        this.referenceHy      = referenceHy;
        this.samplingStep     = samplingStep;

        // find the inner circle centered on (targetHx, targetHy)
        // and osculating the circle corresponding to limit inclination angle
        final double ratio        = FastMath.tan(limitInclination / 2) /
                                    FastMath.hypot(referenceHx, referenceHy);
        final double osculatingHx = ratio * referenceHx;
        final double osculatingHy = ratio * referenceHy;
        innerRadius               = FastMath.hypot(osculatingHx - referenceHx, osculatingHy - referenceHy);

        // rough order of magnitudes values for initialization purposes
        fitted = new double[][] {
            { referenceHx, 0.0,     1.0e-4, 1.0e-4, 1.0e-5, 1.0e-5 },
            { referenceHy, 1.0e-10, 1.0e-4, 1.0e-4, 1.0e-5, 1.0e-5 }
        };

    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws PropagationException {

        this.iteration = iteration;

        // select a long maneuver-free interval for fitting
        final AbsoluteDate[] freeInterval = getManeuverFreeInterval(maneuvers, fixedManeuvers, start, end);

        fitStart = propagator.propagate(freeInterval[0]);
        this.cycleStart = start;

        if (iteration == 0) {
            // reconstruct inclination motion model only on first iteration

            // fit secular plus long periods model to inclination
            CurveFitter xFitter = new CurveFitter(new LevenbergMarquardtOptimizer());
            CurveFitter yFitter = new CurveFitter(new LevenbergMarquardtOptimizer());
            for (AbsoluteDate date = freeInterval[0]; date.compareTo(freeInterval[1]) < 0; date = date.shiftedBy(samplingStep)) {
                final EquinoctialOrbit orbit =
                        (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(propagator.propagate(date).getOrbit());
                final double dt = date.durationFrom(freeInterval[0]);
                xFitter.addObservedPoint(dt, orbit.getHx());
                yFitter.addObservedPoint(dt, orbit.getHy());
            }

            fitted[0] = xFitter.fit(new SecularAndLongPeriod(), fitted[0]);
            fitted[1] = yFitter.fit(new SecularAndLongPeriod(), fitted[1]);

        }

    }

    /** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException {

        final ScheduledManeuver[] tuned;
        final ManeuverAdapterPropagator adapterPropagator = new ManeuverAdapterPropagator(reference);

        if (iteration == 0) {

            if (getMargins() >= 0) {
                // no constraints violations, we don't perform any maneuvers
                nbMan = 0;
                return tunables;
            }

            // we need to first define the number of maneuvers and their initial settings
            final double dt     = cycleStart.shiftedBy(firstOffset).durationFrom(fitStart.getDate());
            final double sunC   = FastMath.cos(SUN_PULSATION  * dt);
            final double sunS   = FastMath.sin(SUN_PULSATION  * dt);
            final double moonC  = FastMath.cos(MOON_PULSATION * dt);
            final double moonS  = FastMath.sin(MOON_PULSATION * dt);

            // inclination vector at maneuver time
            startHx = fitted[0][0]         + fitted[0][1] * dt +
                      fitted[0][2] * sunC  + fitted[0][3] * sunS +
                      fitted[0][4] * moonC + fitted[0][5] * moonS;
            startHy = fitted[1][0]         + fitted[1][1] * dt +
                      fitted[1][2] * sunC  + fitted[1][3] * sunS +
                      fitted[1][4] * moonC + fitted[1][5] * moonS;

            // inclination vector evolution direction, considering only secular and Sun effects
            // we ignore Moon effects here since they have a too short period
            final double dHXdT = fitted[0][1] + SUN_PULSATION  * (fitted[0][3] * sunC  - fitted[0][2] * sunS);
            final double dHYdT = fitted[1][1] + SUN_PULSATION  * (fitted[1][3] * sunC  - fitted[1][2] * sunS);

            // select a target a point on the limit circle
            // such that trajectory enters the circle inner part radially at that point
            targetHx = referenceHx - innerRadius * dHXdT / FastMath.hypot(dHXdT, dHYdT);
            targetHy = referenceHy - innerRadius * dHYdT / FastMath.hypot(dHXdT, dHYdT);

            // compute the out of plane maneuver required to get this inclination offset
            final double deltaHx     = targetHx - startHx;
            final double deltaHy     = targetHy - startHy;
            final double vs          = fitStart.getPVCoordinates().getVelocity().getNorm();
            final double totalDeltaV = thrustSignMomentum(fitStart) * 2 * FastMath.hypot(deltaHx, deltaHy) * vs;

            // compute the number of maneuvers required
            final TunableManeuver model = getModel();
            final double limitDV = (totalDeltaV < 0) ? model.getDVInf() : model.getDVSup();
            nbMan = FastMath.min(maxManeuvers, (int) FastMath.ceil(FastMath.abs(totalDeltaV / limitDV)));
            final double deltaV  = FastMath.max(model.getDVInf(),
                                                FastMath.min(model.getDVSup(), totalDeltaV / nbMan));

            tuned = new ScheduledManeuver[tunables.length + nbMan];
            System.arraycopy(tunables, 0, tuned, 0, tunables.length);
            changeTrajectory(tuned, 0, tunables.length, adapterPropagator);

            final EquinoctialOrbit orbit =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(fitStart.getOrbit());
            final double alphaStart = orbit.getLM();
            final double alphaMan   = FastMath.atan2(deltaHy, deltaHx);
            final double dAlpha     = MathUtils.normalizeAngle(alphaMan - alphaStart, 2 * FastMath.PI);
            final double n          = fitStart.getKeplerianMeanMotion();
            final double separation = orbitsSeparation * 2 * FastMath.PI / n;
            AbsoluteDate t0         = fitStart.getDate().shiftedBy(dAlpha / n);
            while (t0.durationFrom(cycleStart) < firstOffset) {
                t0 = t0.shiftedBy(separation);
            }

            // add the new maneuvers
            for (int i = 0; i < nbMan; ++i) {
                tuned[tunables.length + i] =
                        new ScheduledManeuver(model, t0.shiftedBy(i * separation),
                                              new Vector3D(deltaV, model.getDirection()),
                                              model.getCurrentThrust(), model.getCurrentISP(),
                                              adapterPropagator, false);
            }

        } else {

            if (nbMan == 0) {
                // no maneuvers are needed
                return tunables;
            }

            // adjust the existing maneuvers

            // find the date of the last adjusted maneuver
            ScheduledManeuver last = null;
            for (final ScheduledManeuver maneuver : tunables) {
                if (maneuver.getName().equals(getModel().getName())) {
                    if (last == null || maneuver.getDate().compareTo(last.getDate()) > 0) {
                        last = maneuver;
                    }
                }
            }

            // achieved inclination after the last maneuver
            final EquinoctialOrbit orbit =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(last.getStateAfter().getOrbit());
            final double achievedHx = orbit.getHx();
            final double achievedHy = orbit.getHy();

            // velocity increment correction needed to reach target
            final double ratio  = FastMath.hypot(targetHx   - startHx, targetHy   - startHy) /
                                  FastMath.hypot(achievedHx - startHx, achievedHy - startHy);
            final double deltaV = nbMan * last.getSignedDeltaV() * (ratio - 1.0);

            // time offset needed to reach target
            final double deltaAlpha =
                    MathUtils.normalizeAngle(FastMath.atan2(targetHy   - startHy, targetHx   - startHx) -
                                             FastMath.atan2(achievedHy - startHy, achievedHx - startHx),
                                             0.0);
            final double deltaT = deltaAlpha / orbit.getKeplerianMeanMotion();

            // distribute the change over all maneuvers
            tuned = tunables.clone();
            changeTrajectory(tuned, 0, tuned.length, adapterPropagator);
            distributeDV(deltaV, deltaT, tuned, adapterPropagator);

        }

        // finalize propagator
        for (final ScheduledManeuver maneuver : tuned) {
            adapterPropagator.addManeuver(maneuver.getDate(), maneuver.getDeltaV(), maneuver.getIsp());
        }

        return tuned;

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
        private static final long serialVersionUID = 8803174499877772678L;

        /** {@inheritDoc} */
        public void init(final SpacecraftState s0, final AbsoluteDate t) {
            resetMarginsChecks();
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

                    if (date.compareTo(fitStart.getDate()) > 0) {
                        // compute position in Earth frame
                        interpolator.setInterpolatedDate(date);
                        final SpacecraftState state = interpolator.getInterpolatedState();

                        // check limit circle violations
                        final double inclination = 2 * FastMath.atan(FastMath.hypot(state.getHx(), state.getHy()));
                        checkMargins(FastMath.toDegrees(inclination));
                    }

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

    /** Inner class for secular plus long period motion.
     * <p>
     * This function has 6 parameters, two for the secular part
     * (constant and slope), two for Moon effects and two for Sun
     * effects (cosines and sines).
     * </p>
     */
    private static class SecularAndLongPeriod implements ParametricUnivariateFunction {

        /** {@inheritDoc} */
        public double[] gradient(double x, double ... parameters) {
            return new double[] {
                1.0,                              // constant term of the secular part
                x,                                // slope term of the secular part
                FastMath.cos(SUN_PULSATION  * x), // cosine part of the Sun effect
                FastMath.sin(SUN_PULSATION  * x), // sine part of the Sun effect
                FastMath.cos(MOON_PULSATION * x), // cosine part of the Moon effect
                FastMath.sin(MOON_PULSATION * x)  // sine part of the Moon effect
            };
        }

        /** {@inheritDoc} */
        public double value(final double x, final double ... parameters) {
            return parameters[0] +
                   parameters[1] * x +
                   parameters[2] * FastMath.cos(SUN_PULSATION  * x) +
                   parameters[3] * FastMath.sin(SUN_PULSATION  * x) +
                   parameters[4] * FastMath.cos(MOON_PULSATION * x) +
                   parameters[5] * FastMath.sin(MOON_PULSATION * x);
        }

    }

}
