/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.List;

import org.apache.commons.math.analysis.ParametricUnivariateFunction;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.optimization.fitting.CurveFitter;
import org.apache.commons.math.optimization.general.LevenbergMarquardtOptimizer;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.ManeuverAdapterPropagator;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;

/**
 * Station-keeping control for mean solar time in sun synchronous Low Earth Orbits.
 * <p>
 * The mean solar time at a specific latitude crossinf evolves as a quadratic model
 * of time with medium periods at one day and one half day. The coefficients of this
 * model depend on semi major axis and inclination, since the main effect of
 * J<sub>2</sub> is a secular drift of ascending node with short periodic effects
 * at twice the spacecraft pulsation.
 * </p>
 * <p>
 * The evolution of mean solar time is very slow, it takes several months to exceeds
 * a few minutes offset from central assigned mean solar time. The aim of the control
 * law is to achieve a parabolic motion that stays within the assigned deadband, by
 * having the peak of the parabola (once medium periodic terms have been removed)
 * that osculates the deadband limit.
 * </p>
 * <p>
 * The model parameters are not computed from Keplerian motion, but rather fitted to
 * the real evolution of the parameters against the polynomial and periodic models
 * above. The fitting is performed on the longest maneuver-free interval of the
 * cycle after the last out-of-plane maneuver.
 * </p>
 * <p>
 * The maneuvers which target a parabola osculating the deadband are out-or-plane
 * maneuvers performed at a node (nodes in eclipse are selected) which change
 * inclination and hence change ascending node drift rate, which is directly the
 * slope of the parabola.
 * </p>
 * <p>
 * If the deadband limits are not crossed before next cycle, no maneuvers at all
 * are performed, even if the parabola does not perfectly osculates the deadband but
 * is well inside it. Maneuvers are triggered only when deadband limits will be
 * exceeded before next cycle. If the maneuvers are too large, they will be split
 * into several maneuvers, up to the maximal number of allowed maneuvers par cycle
 * and fulfilling the constraints on velocity increment sizes.
 * </p>
 * @author Luc Maisonobe
 */
public class MeanLocalSolarTime extends AbstractSKControl {

    /** Medium period model pulsation. */
    private static final double BASE_PULSATION = 2.0 * FastMath.PI / Constants.JULIAN_DAY;

    /** Maximum number of maneuvers to set up in one cycle. */
    private final int maxManeuvers;

    /** Time offset of the first maneuver with respect to cycle start. */
    private final double firstOffset;

    /** Minimum time between split parts in number of orbits. */
    private final int orbitsSeparation;

    /** Reference state at first node in eclipse. */
    private SpacecraftState nodeState;

    /** Iteration number. */
    private int iteration;

    /** Cycle end. */
    private AbsoluteDate end;

    /** Latitude at witch the local solar time will be checked*/
    private double latitude;

    /** Earth model. */
    private BodyShape earth;

    /** Indicator for ascending crossing of latitude. */
    private boolean ascending;

    /** Greenwhich mean of date frame. */
    private GMODFrame gmod;

    /** Parameters of the fitted mean solar time model. */
    private double[] fittedH;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model out-of-plane maneuver model
     * @param firstOffset time offset of the first maneuver with respect to cycle start
     * @param maxManeuvers maximum number of maneuvers to set up in one cycle
     * @param orbitsSeparation minimum time between split parts in number of orbits
     * @param earth Earth model
     * @param latitude latitude at which solar time should be computed
     * @param ascending if true, solar time is computed when crossing the
     * specified latitude from south to north
     * @param solarTime target solar time (in fractional hour, i.e 9h30 = 9.5)
     * @param solarTimetolerance solar time tolerance (in hours)
     * @exception OrekitException if the UTC-TAI correction cannot be loaded
     */
    public MeanLocalSolarTime(final String name, final String controlledName, final int controlledIndex,
                              final TunableManeuver model, final double firstOffset,
                              final int maxManeuvers, final int orbitsSeparation,
                              final BodyShape earth, final double latitude, final boolean ascending,
                              final double solarTime, final double solarTimetolerance)
        throws OrekitException {
        super(name, model, controlledName, controlledIndex, null, -1,
              solarTime - solarTimetolerance, solarTime + solarTimetolerance);
        this.firstOffset      = firstOffset;
        this.maxManeuvers     = maxManeuvers;
        this.orbitsSeparation = orbitsSeparation;
        this.latitude         = latitude;
        this.earth            = earth;
        this.ascending        = ascending;
        this.gmod             = new GMODFrame();

        // rough order of magnitudes values for initialization purposes
        fittedH = new double[] {
            solarTime, -1.0e-10, -1.0e-17, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5
        };

    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException, SkatException {

        this.iteration = iteration;
        this.end       = end;
        resetMarginsChecks();

        if (iteration == 0) {

            // select a long maneuver-free interval for fitting
            final AbsoluteDate[] freeInterval = getManeuverFreeInterval(maneuvers, fixedManeuvers, start, end);

            // find the first following node that is in eclipse
            final double period = propagator.getInitialState().getKeplerianPeriod();
            final double stepSize = period / 100;
            nodeState = findFirstCrossing(0.0, true, earth, freeInterval[0].shiftedBy(firstOffset),
                                          end, stepSize, propagator);
            double mst = meanSolarTime(nodeState);
            if (mst >= 6.0 && mst <= 18.0) {
                // wrong node, it is under Sun light, select the next one
                nodeState = findLatitudeCrossing(latitude, earth, nodeState.getDate().shiftedBy(0.5 * period),
                                                 end, stepSize, period / 8, propagator);
            }

        }

        // fit parabolic plus medium periods model to mean solar time
        CurveFitter mstFitter = new CurveFitter(new LevenbergMarquardtOptimizer());

        // find the first latitude crossing
        double period   = nodeState.getKeplerianPeriod();
        double stepSize = period / 100;
        SpacecraftState crossing =
                findFirstCrossing(latitude, ascending, earth, nodeState.getDate(), end, stepSize, propagator);
        double mst = meanSolarTime(crossing);
        checkMargins(mst);
        double dt = crossing.getDate().durationFrom(start);
        mstFitter.addObservedPoint(dt, mst);

        // find all other latitude crossings from regular schedule
        while (crossing != null && crossing.getDate().shiftedBy(period).compareTo(end) < 0) {

            final AbsoluteDate previous = crossing.getDate();
            crossing = findLatitudeCrossing(latitude, earth, previous.shiftedBy(period),
                                            end, stepSize, period / 8, propagator);
            if (crossing != null) {

                // Store current point
                mst = meanSolarTime(crossing);
                checkMargins(mst);
                dt = crossing.getDate().durationFrom(start);
                mstFitter.addObservedPoint(dt, mst);

                // use the same time separation to pinpoint next crossing
                period = crossing.getDate().durationFrom(previous);

            }

        }

        fittedH = mstFitter.fit(new ParabolicAndMediumPeriod(), fittedH);

    }

    /** Compute the mean solar time.
     * @param state current spacecraft state
     * @return mean solar time
     * @exception OrekitException if state cannot be converted
     */
    private double meanSolarTime(final SpacecraftState state)
        throws OrekitException {

        // compute angle between Sun and spacecraft in the equatorial plane
        final Vector3D position = state.getPVCoordinates().getPosition();
        final double time       = state.getDate().getComponents(TimeScalesFactory.getUTC()).getTime().getSecondsInDay();
        final double gmst       = gmod.getMeanSiderealTime(state.getDate());
        final double sunAlpha   = gmst + FastMath.PI * (1 - time / (Constants.JULIAN_DAY * 0.5));
        final double dAlpha     = MathUtils.normalizeAngle(position.getAlpha() - sunAlpha, 0);

        // convert the angle to solar time
        return 12.0 * (1.0 + dAlpha / FastMath.PI);

    }

    /** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException {

        // upper bound of mean solar time allocation due to medium period effects
        final double harmonicAllocation =
                2 * (FastMath.max(FastMath.abs(fittedH[3]), FastMath.abs(fittedH[4])) +
                     FastMath.max(FastMath.abs(fittedH[5]), FastMath.abs(fittedH[6])));

        // remaining part of the mean local time window for parabolic motion
        final double hMax = getMax() - 0.5 * harmonicAllocation;
        final double hMin = getMin() + 0.5 * harmonicAllocation;

        // which depends on current state
        final double newHdot;
        if ((fittedH[2] < 0 && fittedH[0] > hMax) || (fittedH[2] > 0 && fittedH[0] < hMin)) {
            // the start point is already on the wrong side of the window

            // the current cycle is already bad, we set up a target to start a new cycle
            // at time horizon with good initial conditions, and reach this target by changing hDot(t0)
            final double targetT = end.durationFrom(nodeState.getDate()) + firstOffset;
            final double targetH = (fittedH[2] < 0) ? hMin : hMax;
            newHdot = (targetH - fittedH[0]) / targetT - fittedH[2] * targetT;

        } else {
            // the start point is on the right side of the window

            final double tPeak   = -0.5 * fittedH[1] / fittedH[2];
            final double hPeak   = fittedH[0] + 0.5 * fittedH[1] * tPeak;
            final double finalT = end.durationFrom(nodeState.getDate()) + firstOffset;
            final double finalH = fittedH[0] + finalT * (fittedH[1] + finalT * fittedH[2]);

            final boolean intermediateExit = tPeak > 0 && tPeak < finalT && (hPeak > hMax || hPeak < hMin);
            final boolean finalExit        = finalH >= hMax || finalH <= hMin;
            if (intermediateExit || finalExit) {
                // mean solar time exits the window limit before next cycle
                final boolean exitBeforePeak = intermediateExit || (finalExit && finalT < tPeak);

                // we target a future mean solar time peak osculating window boundary
                final double targetH = (fittedH[2] <= 0) ? hMax : hMin;
                newHdot = FastMath.copySign(FastMath.sqrt(4 * fittedH[2] * (fittedH[0] - targetH)),
                                            exitBeforePeak ? fittedH[1] : -fittedH[1]);

            } else {
                // mean solar time stays within bounds up to next cycle,
                // we don't change anything on the maneuvers
                return tunables;
            }

        }

        // linearized relationship between hDot and inclination in the neighborhood or maneuver
        // hDot = alphaDot - raanDot where alphaDot is the angular rate of Sun right ascension
        // and raanDot is the angular rate of ascending node right ascension. So hDot evolution
        // is the opposite of raan evolution, which itself is proportional to cos(i) due to J2
        // effect. So hDot = alphaDot - k cos(i) and hence Delta hDot = -k sin(i) Delta i
        // so Delta hDot / Delta i = (alphaDot - hDot) tan(i)
        final double dhDotDi = (2 * FastMath.PI / Constants.JULIAN_YEAR - fittedH[1]) * FastMath.tan(nodeState.getI());

        // compute inclination offset needed to achieve station-keeping target
        final double deltaI = (newHdot - fittedH[1]) / dhDotDi;

        final ScheduledManeuver[] tuned;
        final ManeuverAdapterPropagator adapterPropagator = new ManeuverAdapterPropagator(reference);
        if (iteration == 0) {
            // we need to first define the number of maneuvers and their initial settings

            // compute the out of plane maneuver required to get the initial inclination offset
            final Vector3D v          = nodeState.getPVCoordinates().getVelocity();
            final double totalDeltaV  = thrustSignMomentum(nodeState) * FastMath.signum(v.getZ()) * 2 * v.getNorm() * deltaI;

            // compute the number of maneuvers required
            final TunableManeuver model = getModel();
            final double limitDV = (totalDeltaV < 0) ? model.getDVInf() : model.getDVSup();
            final int    nMan    = FastMath.min(maxManeuvers,
                                                (int) FastMath.ceil(FastMath.abs(totalDeltaV / limitDV)));
            final double deltaV  = FastMath.max(model.getDVInf(),
                                                FastMath.min(model.getDVSup(), totalDeltaV / nMan));

            tuned = new ScheduledManeuver[tunables.length + nMan];
            System.arraycopy(tunables, 0, tuned, 0, tunables.length);
            changeTrajectory(tuned, 0, tunables.length, adapterPropagator);

            final double separation = orbitsSeparation * nodeState.getKeplerianPeriod();

            // add the new maneuvers
            for (int i = 0; i < nMan; ++i) {
                tuned[tunables.length + i] =
                        new ScheduledManeuver(model, nodeState.getDate().shiftedBy(i * separation),
                                              new Vector3D(deltaV, model.getDirection()),
                                              model.getCurrentThrust(), model.getCurrentISP(),
                                              adapterPropagator, false);
            }

        } else {

            // adjust the existing maneuvers

            // compute the out of plane maneuver required to get the initial inclination offset
            final double v            = nodeState.getPVCoordinates().getVelocity().getNorm();
            final double deltaVChange = thrustSignMomentum(nodeState) * 2 * v * deltaI;

            // distribute the change over all maneuvers
            tuned = tunables.clone();
            changeTrajectory(tuned, 0, tuned.length, adapterPropagator);
            distributeDV(deltaVChange, 0.0, tuned, adapterPropagator);

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
        return null;
    }

    /** Inner class for parabolic plus medium period motion.
     * <p>
     * This function has 5 parameters, three for the parabolic part
     * and two for the cosine and sine.
     * </p>
     */
    private static class ParabolicAndMediumPeriod implements ParametricUnivariateFunction {

        /** {@inheritDoc} */
        public double[] gradient(double x, double ... parameters) {
            return new double[] {
                1.0,                                  // constant term of the parabolic part
                x,                                    // slope term of the parabolic part
                x * x,                                // quadratic term of the parabolic part
                FastMath.cos(BASE_PULSATION * x),     // cosine part of the first medium period part
                FastMath.sin(BASE_PULSATION * x),     // sine part of the first medium period part
                FastMath.cos(2 * BASE_PULSATION * x), // cosine part of the second medium period part
                FastMath.sin(2 * BASE_PULSATION * x)  // sine part of the second medium period part
            };
        }

        /** {@inheritDoc} */
        public double value(final double x, final double ... parameters) {
            return parameters[0] +
                   parameters[1] * x +
                   parameters[2] * x  * x +
                   parameters[3] * FastMath.cos(BASE_PULSATION * x) +
                   parameters[4] * FastMath.sin(BASE_PULSATION * x) +
                   parameters[5] * FastMath.cos(2 * BASE_PULSATION * x) +
                   parameters[6] * FastMath.sin(2 * BASE_PULSATION * x);
        }

    }

}
