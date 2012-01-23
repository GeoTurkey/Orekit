/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.AdapterPropagator;
import org.orekit.propagation.analytical.J2DifferentialEffect;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.SecularAndHarmonic;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;

/**
 * Station-keeping control for mean solar time in sun synchronous Low Earth Orbits.
 * <p>
 * The mean solar time at a specific latitude crossing evolves as a quadratic model
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

    /** Long period model pulsation. */
    private static final double SUN_PULSATION = 2.0 * FastMath.PI / Constants.JULIAN_YEAR;

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

    /** Latitude at witch the local solar time will be checked*/
    private double latitude;

    /** Earth model. */
    private BodyShape earth;

    /** Reference radius of the Earth for the potential model. */
    private final double referenceRadius;

    /** Central attraction coefficient. */
    private double mu;

    /** Un-normalized zonal coefficient. */
    private double j2;

    /** Indicator for ascending crossing of latitude. */
    private boolean ascending;

    /** Greenwhich mean of date frame. */
    private GMODFrame gmod;

    /** Mean solar time model. */
    private SecularAndHarmonic mstModel;

    /** Start of current cycle. */
    private AbsoluteDate cycleStart;

    /** End of current cycle. */
    private AbsoluteDate cycleEnd;

    /** Maneuver-free interval. */
    private AbsoluteDate[] freeInterval;

    /** Propagator. */
    private Propagator propagator;

    /** Dates of latitude crossings. */
    private final List<AbsoluteDate> crossings;

    /** Safety margin on mean solar time window. */
    private final double safetyMargin;

    /** Indicator for compensating long burns inefficiency. */
    private boolean compensateLongBurn;

    /** Indicator for side targeting. */
    private boolean forceReset;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model out-of-plane maneuver model
     * @param firstOffset time offset of the first maneuver with respect to cycle start
     * @param maxManeuvers maximum number of maneuvers to set up in one cycle
     * @param orbitsSeparation minimum time between split parts in number of orbits
     * @param earth Earth model
     * @param referenceRadius reference radius of the Earth for the potential model (m)
     * @param mu central attraction coefficient (m<sup>3</sup>/s<sup>2</sup>)
     * @param j2 un-normalized zonal coefficient (about +1.08e-3 for Earth)
     * @param latitude latitude at which solar time should be computed
     * @param ascending if true, solar time is computed when crossing the
     * specified latitude from south to north
     * @param solarTime target solar time (in fractional hour, i.e 9h30 = 9.5)
     * @param solarTimetolerance solar time tolerance (in hours)
     * @param horizon time horizon duration
     * @param compensateLongBurn if true, long burn inefficiency should be compensated
     * @exception OrekitException if the UTC-TAI correction cannot be loaded
     */
    public MeanLocalSolarTime(final String name, final String controlledName, final int controlledIndex,
                              final TunableManeuver model, final double firstOffset,
                              final int maxManeuvers, final int orbitsSeparation, final BodyShape earth,
                              final double referenceRadius, final double mu, final double j2,
                              final double latitude, final boolean ascending,
                              final double solarTime, final double solarTimetolerance, final double horizon,
                              final boolean compensateLongBurn)
       throws OrekitException {
        super(name, model, controlledName, controlledIndex, null, -1,
              solarTime - solarTimetolerance, solarTime + solarTimetolerance,
              horizon * Constants.JULIAN_DAY);
        this.firstOffset        = firstOffset;
        this.maxManeuvers       = maxManeuvers;
        this.orbitsSeparation   = orbitsSeparation;
        this.latitude           = latitude;
        this.earth              = earth;
        this.referenceRadius    = referenceRadius;
        this.mu                 = mu;
        this.j2                 = j2;
        this.ascending          = ascending;
        this.gmod               = new GMODFrame();
        this.safetyMargin       = 0.1 * solarTimetolerance;
        this.compensateLongBurn = compensateLongBurn;

        mstModel = new SecularAndHarmonic(2,
                                          new double[] {
                                              SUN_PULSATION, 2 * SUN_PULSATION,
                                              BASE_PULSATION, 2 * BASE_PULSATION
                                          });

        // rough order of magnitudes values for initialization purposes
        mstModel.resetFitting(AbsoluteDate.J2000_EPOCH,
                              new double[] {
                                  solarTime, -1.0e-10, -1.0e-17,
                                  1.0e-3, 1.0e-3, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5
                              });

        crossings = new ArrayList<AbsoluteDate>();

    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException, SkatException {

        this.iteration   = iteration;
        this.propagator  = propagator;
        this.cycleStart  = start;
        this.cycleEnd    = end;
        resetMarginsChecks();
        crossings.clear();

        if (iteration == 0) {
            forceReset = false;
        }

        // select a long maneuver-free interval for fitting
        freeInterval    = getManeuverFreeInterval(maneuvers, fixedManeuvers, start, end);
        double period   = propagator.getInitialState().getKeplerianPeriod();
        double stepSize = period / 100.0;
        if (freeInterval[1].durationFrom(freeInterval[0]) < period) {
            // if free interval is too short, use only one checking value
            AbsoluteDate t1 = start.shiftedBy(FastMath.max(0, freeInterval[0].durationFrom(start) - period));
            AbsoluteDate t2 = t1.shiftedBy(2 * period);
            if (t2.compareTo(start) >= getTimeHorizon()) {
                t2 = start.shiftedBy(getTimeHorizon());
            }
            SpacecraftState crossing =
                    firstLatitudeCrossing(latitude, ascending, earth, t1, t2, stepSize, propagator);
            double mst = meanSolarTime(crossing);
            mstModel.resetFitting(freeInterval[0], mstModel.getFittedParameters());
            mstModel.addPoint(crossing.getDate(), mst);
            checkMargins(crossing.getDate(), mst);
            return;
        }

        // fit the mean solar time model
        SpacecraftState crossing =
                firstLatitudeCrossing(latitude, ascending, earth, freeInterval[0], freeInterval[1], stepSize, propagator);
        double mst = meanSolarTime(crossing);
        mstModel.resetFitting(freeInterval[0], mstModel.getFittedParameters());
        mstModel.addPoint(crossing.getDate(), mst);
        checkMargins(crossing.getDate(), mst);

        // find all other latitude crossings from regular schedule
        while (crossing != null && crossing.getDate().shiftedBy(period).compareTo(freeInterval[1]) < 0) {

            final AbsoluteDate previous = crossing.getDate();
            crossing = latitudeCrossing(latitude, earth, previous.shiftedBy(period),
                                        freeInterval[1], stepSize, period / 8, propagator);
            if (crossing != null) {

                // Store current point
                crossings.add(crossing.getDate());
                mst = meanSolarTime(crossing);
                mstModel.addPoint(crossing.getDate(), mst);
                checkMargins(crossing.getDate(), mst);

                // use the same time separation to pinpoint next crossing
                period = crossing.getDate().durationFrom(previous);

            }

        }
        mstModel.fit();

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

        final double hMin   = getMin();
        final double hMax   = getMax();
        final double period = crossings.get(1).durationFrom(crossings.get(0));

        if (iteration == 0) {

            // look for a boundaries violation
            nodeState = null;
            for (int i = 0; nodeState == null && i < crossings.size(); ++i) {

                final double crossingMst = mstModel.meanValue(crossings.get(i), 2, 2);
                if (crossingMst <= hMin || crossingMst >= hMax) {
                    // find the first available node that is in eclipse for maneuver
                    final double stepSize = period / 100;
                    AbsoluteDate search = crossings.get(i).shiftedBy(Constants.JULIAN_DAY);
                    if (search.durationFrom(cycleStart) <= period) {
                        search = cycleStart.shiftedBy(period);
                    }
                    if (cycleEnd.durationFrom(search) >= period) {
                        nodeState = latitudeCrossing(latitude, earth, search, freeInterval[1],
                                                     stepSize, period, propagator);
                        final double nodeMst = meanSolarTime(nodeState);
                        if (nodeMst >= 6.0 && nodeMst <= 18.0) {
                            // wrong node, it is under Sun light, select the next one
                            nodeState = latitudeCrossing(latitude, earth, nodeState.getDate().shiftedBy(0.5 * period),
                                                         freeInterval[1], stepSize, period / 8, propagator);
                        }

                        // ensure maneuvers separation requirements
                        while (nodeState.getDate().durationFrom(freeInterval[0]) < firstOffset) {
                            nodeState = latitudeCrossing(latitude, earth, nodeState.getDate().shiftedBy(period),
                                                         freeInterval[1], stepSize, period / 8, propagator);
                        }
                    }

                }

            }

        }

        if (nodeState == null) {
            // no maneuvers needed
            return tunables;
        }

        // simplify mean model to a single quadratic (this includes the
        // 6 months long period, which may be larger than the pure secular terms
        // at some times)
        final double[] mst = mstModel.approximateAsPolynomialOnly(2, nodeState.getDate(), 2, 2,
                                                                  nodeState.getDate(), freeInterval[1], period);

        double newHdot;
        if (forceReset || (mst[2] < 0 && mst[0] > hMax) || (mst[2] > 0 && mst[0] < hMin)) {
            // the start point is already on the wrong side of the window

            // make sure once we select this option, we stick to it for all iterations
            forceReset = true;

            // the current cycle is already bad, we set up a target to start a new cycle
            // at time horizon with good initial conditions, and reach this target by changing hDot(t0)
            final double targetT = cycleEnd.durationFrom(nodeState.getDate()) + firstOffset;
            final double targetH = (mst[2] < 0) ? (hMin + safetyMargin) : (hMax - safetyMargin);
            newHdot = (targetH - mst[0]) / targetT - mst[2] * targetT;

        } else {
            // the start point is on the right side of the window
            final double tPeak   = -0.5 * mst[1] / mst[2];
            final double hPeak   = mst[0] + 0.5 * mst[1] * tPeak;
            final double finalT = cycleEnd.durationFrom(nodeState.getDate()) + firstOffset;
            final double finalH = mst[0] + finalT * (mst[1] + finalT * mst[2]);

            final boolean intermediateExit = tPeak > 0 && tPeak < finalT && (hPeak > hMax || hPeak < hMin);
            final boolean finalExit        = finalH >= hMax || finalH <= hMin;
            if (intermediateExit || finalExit) {
                // mean solar time exits the window limits

                // we target a future mean solar time peak osculating window boundary
                // (taking a safety margin into account if possible)
                double targetH;
                if (mst[2] <= 0) {
                    targetH = hMax - safetyMargin;
                    if (mst[0] >= targetH) {
                        targetH = hMax;
                    }
                } else {
                    targetH = hMin + safetyMargin;
                    if (mst[0] <= targetH) {
                        targetH = hMin;
                    }
                }
                newHdot = FastMath.copySign(FastMath.sqrt(4 * mst[2] * (mst[0] - targetH)),
                                            (tPeak > 0) ? mst[1] : -mst[1]);

            } else {
                // mean solar time stays within bounds up to next cycle,
                // we don't change anything on the maneuvers
                nodeState = null;
                return tunables;
            }

        }

        // linearized relationship between hDot and inclination in the neighborhood of maneuver
        // the solar time h(t) is approximately h(tn) = Omega(tn) + eta - (pi + alpha(tn))
        // where Omega is spacecraft right ascension of ascending node, eta is right ascension
        // difference between spacecraft at specified latitude and ascending node, and alpha is
        // Sun right ascension. Eta and pi are fixed, so we get the time derivative
        // hDot = raanDot - alphaDot. Raan time derivative due to J2 is proportional to cos(i),
        // so hDot = k cos(i) - alphaDot (with both k and cos(i) negative for sun synchronous
        // orbits so hDot is close to zero) and hence dhDot / di = -k sin (i) = -(alphaDot + hDot) tan(i)
        final double dhRadDotDi = -(2 * FastMath.PI / Constants.JULIAN_YEAR + mst[1]) * FastMath.tan(nodeState.getI());
        final double dhDotDi    = 12 * dhRadDotDi / FastMath.PI;

        // compute inclination offset needed to achieve station-keeping target
        final double deltaI = (newHdot - mst[1]) / dhDotDi;

        final ScheduledManeuver[] tuned;
        final AdapterPropagator adapterPropagator = new AdapterPropagator(reference);
        if (iteration == 0) {
            // we need to first define the number of maneuvers and their initial settings

            // compute the out of plane maneuver required to get the initial inclination offset
            final Vector3D v          = nodeState.getPVCoordinates().getVelocity();
            final double totalDeltaV  = thrustSignMomentum(nodeState) * FastMath.signum(v.getZ()) *
                                        v.getNorm() * deltaI;

            // compute the number of maneuvers required
            final double separation = orbitsSeparation * nodeState.getKeplerianPeriod();
            final TunableManeuver model = getModel();
            final double limitDV = (totalDeltaV < 0) ? model.getDVInf() : model.getDVSup();
            int nMan = FastMath.min(maxManeuvers, (int) FastMath.ceil(FastMath.abs(totalDeltaV / limitDV)));
            while (nodeState.getDate().shiftedBy((nMan - 1) * separation).getDate().compareTo(cycleEnd) >= 0) {
                --nMan;
            }
            final double deltaV = FastMath.max(model.getDVInf(), FastMath.min(model.getDVSup(), totalDeltaV / nMan));

            tuned = new ScheduledManeuver[tunables.length + nMan];
            System.arraycopy(tunables, 0, tuned, 0, tunables.length);
            changeTrajectory(tuned, 0, tunables.length, adapterPropagator);


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
            final double deltaVChange = thrustSignMomentum(nodeState) * v * deltaI;

            // distribute the change over all maneuvers
            tuned = tunables.clone();
            changeTrajectory(tuned, 0, tuned.length, adapterPropagator);
            distributeDV(deltaVChange, 0.0, tuned, adapterPropagator);

        }

        if (compensateLongBurn) {
            for (int i = 0; i < tuned.length; ++i) {
                if (tuned[i].getName().equals(getModel().getName())) {
                    tuned[i] = longBurnCompensation(tuned[i]);
                }
            }
        }

        // finalize propagator
        for (final ScheduledManeuver maneuver : tuned) {
            final SpacecraftState state = maneuver.getStateBefore();
            AdapterPropagator.DifferentialEffect directEffect =
                    new SmallManeuverAnalyticalModel(state, maneuver.getDeltaV(), maneuver.getIsp());
            AdapterPropagator.DifferentialEffect resultingEffect =
                    new J2DifferentialEffect(state, directEffect, false, referenceRadius, mu, j2);
            adapterPropagator.addEffect(directEffect);
            adapterPropagator.addEffect(resultingEffect);
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

    /** Compensate inefficiency of long burns.
     * @param maneuver maneuver to compensate
     * @return compensated maneuver (Isp reduced to get same dV with more consumed mass)
     * @throws PropagationException if state cannot be propagated around maneuvera
     */
    private ScheduledManeuver longBurnCompensation(final ScheduledManeuver maneuver)
        throws PropagationException {
        // this is a long out of plane maneuver, we adapt Isp to reflect
        // the fact more mass will be consumed to achieve the same velocity increment
        final double nominalDuration = maneuver.getDuration(maneuver.getStateBefore().getMass());

        final SpacecraftState startState = maneuver.getState(-0.5 * nominalDuration);
        final CircularOrbit startOrbit   = (CircularOrbit) (OrbitType.CIRCULAR.convertType(startState.getOrbit()));
        final double alphaS              = startOrbit.getAlphaV();

        final SpacecraftState endState   = maneuver.getState(+0.5 * nominalDuration);
        final CircularOrbit endOrbit     = (CircularOrbit) (OrbitType.CIRCULAR.convertType(endState.getOrbit()));
        final double alphaE              = endOrbit.getAlphaV();

        final double reductionFactor = (FastMath.sin(alphaE) - FastMath.sin(alphaS)) / (alphaE - alphaS);
        return new ScheduledManeuver(maneuver.getModel(), maneuver.getDate(),
                                     maneuver.getDeltaV(), maneuver.getThrust(),
                                     reductionFactor * getModel().getCurrentISP(),
                                     maneuver.getTrajectory(), maneuver.isReplanned());
    }

}
