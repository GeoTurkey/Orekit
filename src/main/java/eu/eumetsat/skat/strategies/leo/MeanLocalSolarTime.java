/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BracketingNthOrderBrentSolver;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.optimization.GoalType;
import org.apache.commons.math3.optimization.fitting.PolynomialFitter;
import org.apache.commons.math3.optimization.general.LevenbergMarquardtOptimizer;
import org.apache.commons.math3.optimization.univariate.BrentOptimizer;
import org.apache.commons.math3.optimization.univariate.UnivariateOptimizer;
import org.apache.commons.math3.optimization.univariate.UnivariatePointValuePair;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.MathUtils;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.DateTimeComponents;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Station-keeping control for mean solar time in sun synchronous Low Earth Orbits.
 * <p>
 * The mean solar time at a specific latitude crossing evolves as a quadratic model of time with
 * medium periods at one day and one half day. The coefficients of this model depend on semi major
 * axis and inclination, since the main effect of J<sub>2</sub> is a secular drift of ascending node
 * with short periodic effects at twice the spacecraft pulsation.
 * </p>
 * <p>
 * The evolution of mean solar time is very slow, it takes several months to exceeds a few minutes
 * offset from central assigned mean solar time. The aim of the control law is to achieve a
 * parabolic motion that stays within the assigned deadband, by having the peak of the parabola
 * (once medium periodic terms have been removed) that osculates the deadband limit.
 * </p>
 * <p>
 * The model parameters are not computed from Keplerian motion, but rather fitted to the real
 * evolution of the parameters against the polynomial and periodic models above. The fitting is
 * performed on the longest maneuver-free interval of the cycle after the last out-of-plane
 * maneuver.
 * </p>
 * <p>
 * The maneuvers which target a parabola osculating the deadband are out-or-plane maneuvers
 * performed at a node (nodes in eclipse are selected) which change inclination and hence change
 * ascending node drift rate, which is directly the slope of the parabola.
 * </p>
 * <p>
 * If the deadband limits are not crossed before next cycle, no maneuvers at all are performed, even
 * if the parabola does not perfectly osculates the deadband but is well inside it. Maneuvers are
 * triggered only when deadband limits will be exceeded before next cycle. If the maneuvers are too
 * large, they will be split into several maneuvers, up to the maximal number of allowed maneuvers
 * par cycle and fulfilling the constraints on velocity increment sizes.
 * </p>
 * 
 * @author Luc Maisonobe
 */
public class MeanLocalSolarTime extends AbstractLeoSKControl {

    /** Tropical year duration. */
    private static final double TROPICAL_YEAR = 365.242199 * Constants.JULIAN_DAY;

    /** Latitude at witch the local solar time will be checked */
    private final double latitude;

    /** Indicator for ascending crossing of latitude. */
    private final boolean ascending;

    /** Indicator for compensating long burns inefficiency. */
    private final boolean compensateLongBurn;

    /** Greenwhich mean of date frame. */
    private final GMODFrame gmod;

    /** Reference state at first node in eclipse. */
    private SpacecraftState nodeState;

    /** Reference date for analytical models. */
    private AbsoluteDate tRef;

    /** Reference offset from SSO inclination. */
    private double iOffsetRef;

    /** Reference Mean Local Solar Time for cycle. */
    private double mlstRef;

    /** Propagator. */
    private Propagator propagator;

    /** dHdot / di. */
    private final double dhDotDi;

    /** Maneuvers Day Of Year. */
    private final int[] maneuversDoy;

    /** Analytical models for Mean Local Solar Time. */
    private final MlstModel mlstModel;

    /** Targeting function. */
    private UnivariateFunction targeting;

    /** Scaling factor between offset and slope. */
    private final double scaling;

    /** Interface for providing Mean Local Solar Time analytical model. */
    public interface MlstModel {

        /** Check if inclination increases or decreases with time.
         * @return true if inclination increases or decreases with time
         */
        boolean increasingInclination();

        /** Compute Mean Local Solar Time.
         * @param t current date
         * @param tRef reference date
         * @param dhRadDotDi dHdot / di
         * @param mlstRef reference Mean Local Solar Time at tRef
         * @param iOffsetRef reference offset from SSO inclination
         * @return Mean Local Solar Time in hours at {@code t}
         */
        double mlst(AbsoluteDate t, AbsoluteDate tRef,
                    double dhRadDotDi, double mlstRef, final double iOffsetRef);

    }

    /**
     * Simple constructor.
     * 
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model out-of-plane maneuver model
     * @param firstOffset time offset of the first maneuver with respect to cycle start
     * @param maxManeuvers maximum number of maneuvers to set up in one cycle
     * @param orbitsSeparation minimum time between split parts in number of orbits
     * @param earth Earth model
     * @param sun Sun model
     * @param referenceRadius reference radius of the Earth for the potential model (m)
     * @param mu central attraction coefficient (m<sup>3</sup>/s<sup>2</sup>)
     * @param j2 un-normalized zonal coefficient (about +1.08e-3 for Earth)
     * @param latitude latitude at which solar time should be computed
     * @param ascending if true, solar time is computed when crossing the specified latitude from south to north
     * @param solarTime target solar time (in fractional hour, i.e 9h30 = 9.5)
     * @param solarTimetolerance solar time tolerance (in hours)
     * @param horizon time horizon duration
     * @param compensateLongBurn if true, long burn inefficiency should be compensated
     * @param phasingDays number of days of the phasing cycle
     * @param phasingOrbits number of orbits of the phasing cycle
     * @param maneuversDoy maneuvers Day Of Year
     * @param analyticalModels analytical models for inclination offset and MLST
     * @exception OrekitException if the UTC-TAI correction cannot be loaded
     */
    public MeanLocalSolarTime(final String name, final String controlledName, final int controlledIndex,
                              final TunableManeuver model, final double firstOffset, final int maxManeuvers,
                              final int orbitsSeparation, final OneAxisEllipsoid earth, final CelestialBody sun,
                              final double referenceRadius, final double mu, final double j2,
                              final double latitude, final boolean ascending,
                              final double solarTime, final double solarTimetolerance, final double horizon,
                              final boolean compensateLongBurn, final int phasingDays, final int phasingOrbits,
                              final int[] maneuversDoy, final MlstModel analyticalModels)
        throws OrekitException {

        super(name, controlledName, controlledIndex, model,
              firstOffset, maxManeuvers, orbitsSeparation, earth, sun, referenceRadius, mu, j2,
              solarTime - solarTimetolerance, solarTime + solarTimetolerance, horizon * Constants.JULIAN_DAY);

        this.latitude  = latitude;
        this.ascending = ascending;
        this.gmod = new GMODFrame();
        this.compensateLongBurn = compensateLongBurn;
        this.maneuversDoy = maneuversDoy.clone();
        Arrays.sort(this.maneuversDoy);
        this.mlstModel = analyticalModels;

        // theoretical (i.e. non-fitted) coefficients
        meanPeriod        = phasingDays * Constants.JULIAN_DAY / phasingOrbits;
        final double n    = 2 * FastMath.PI / meanPeriod;
        final double cSSO = -1.5 * j2 * referenceRadius * referenceRadius *
                            n * n * FastMath.cbrt(n / (mu * mu));
        final double iSSO = FastMath.acos(2 * FastMath.PI / (TROPICAL_YEAR * cSSO));
        scaling           = 2;  // TODO this required scaling factor is unexplained ...
        dhDotDi           = -12 * cSSO * FastMath.sin(iSSO) / (scaling * FastMath.PI);

    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration,
                              final ScheduledManeuver[] maneuvers,
                              final Propagator propagator,
                              final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start,
                              final AbsoluteDate end) throws OrekitException, SkatException {

        resetMarginsChecks();
        this.iteration  = iteration;
        this.cycleStart = start;
        this.cycleEnd   = end;
        this.propagator = propagator;

        if (iteration == 0) {
            nodeState = null;
        }

        // select a long maneuver-free interval for fitting
        freeInterval = getManeuverFreeInterval(maneuvers, fixedManeuvers, start, end);
        double stepSize = meanPeriod / 100.0;
        if (freeInterval[1].durationFrom(freeInterval[0]) < meanPeriod) {
            // if free interval is too short, use only one checking value
            AbsoluteDate t1 = start.shiftedBy(FastMath.max(0, freeInterval[0].durationFrom(start) - meanPeriod));
            AbsoluteDate t2 = t1.shiftedBy(2 * meanPeriod);
            if (t2.durationFrom(start) >= getTimeHorizon()) {
                t2 = start.shiftedBy(getTimeHorizon());
            }
            SpacecraftState crossing = firstLatitudeCrossing(latitude, ascending, earth, t1, t2, stepSize, propagator);
            mlstRef    = meanSolarTime(crossing);
            iOffsetRef = 0;
            checkMargins(crossing.getDate(), mlstRef);
            return;
        }

        // find all crossings
        List<SpacecraftState> crossings = new ArrayList<SpacecraftState>();
        SpacecraftState crossing = firstLatitudeCrossing(latitude, ascending, earth,
                                                         start, end, stepSize, propagator);
        checkMargins(crossing.getDate(), meanSolarTime(crossing));
        if (crossing.getDate().compareTo(freeInterval[0]) >= 0 &&
            crossing.getDate().compareTo(freeInterval[1]) <= 0) {
            crossings.add(crossing);
        }

        // find all other latitude crossings from regular schedule
        double deltaT = meanPeriod;
        while (crossing != null && crossing.getDate().shiftedBy(deltaT).compareTo(end) < 0) {

            final AbsoluteDate previous = crossing.getDate();
            crossing = latitudeCrossing(latitude, earth, previous.shiftedBy(deltaT), end, stepSize, meanPeriod / 8, propagator);
            if (crossing != null) {
                checkMargins(crossing.getDate(), meanSolarTime(crossing));
                if (crossing.getDate().compareTo(freeInterval[0]) >= 0 &&
                    crossing.getDate().compareTo(freeInterval[1]) <= 0) {
                    crossings.add(crossing);
                }
                deltaT = crossing.getDate().durationFrom(previous);
            }

        }

        // set up reference date for analytical models
        tRef = (nodeState != null) ? nodeState.getDate() : start;

        // fit the mean solar time constants of integration
        PolynomialFitter fitter = new PolynomialFitter(1, new LevenbergMarquardtOptimizer());
        for (int i = 0; i < crossings.size(); ++i) {
            crossing = crossings.get(i);
            fitter.addObservedPoint(crossing.getDate().durationFrom(tRef),
                                    meanSolarTime(crossing) -
                                    mlstModel.mlst(crossing.getDate(), tRef, dhDotDi, 0.0, 0.0));
        }
        final double[] coefficients = fitter.fit();
        mlstRef    = coefficients[0];
        iOffsetRef = coefficients[1] / dhDotDi;

    }

    /** Compute the mean solar time.
     * 
     *  @param state current spacecraft state
     *  @return mean solar time
     *  @exception OrekitException if state cannot be converted
     */
    private double meanSolarTime(final SpacecraftState state) throws OrekitException {

        // compute angle between Sun and spacecraft in the equatorial plane
        final Vector3D position = state.getPVCoordinates().getPosition();
        final double time = state.getDate().getComponents(TimeScalesFactory.getUTC()).getTime().getSecondsInDay();
        final double gmst = gmod.getMeanSiderealTime(state.getDate());
        final double sunAlpha = gmst + FastMath.PI * (1 - time / (Constants.JULIAN_DAY * 0.5));
        final double dAlpha = MathUtils.normalizeAngle(position.getAlpha() - sunAlpha, 0);

        // convert the angle to solar time
        return 12.0 * (1.0 + dAlpha / FastMath.PI);

    }

    /** Find a min/max MLST value.
     * @param tMin search start
     * @param tMax search end
     * @param searchMin if true, search a min value, otherwise search for a max value
     * @return extrememum (with reference date at tMin)
     */
    private UnivariatePointValuePair extremumMLST(final AbsoluteDate tMin, final AbsoluteDate tMax,
                                                  final boolean searchMin) {

        final UnivariateFunction mlst = new UnivariateFunction() {
            /** {@inheritDoc} */
            public double value(double x) {
                return mlstModel.mlst(tMin.shiftedBy(x), tRef, dhDotDi, mlstRef, iOffsetRef);
            }
        };

        final UnivariateOptimizer optimizer = new BrentOptimizer(1.0e-10, 1.0);

        return optimizer.optimize(1000, mlst, searchMin ? GoalType.MINIMIZE : GoalType.MAXIMIZE,
                                  0, tMax.durationFrom(tMin));

    }

    /** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException, SkatException {

        // find next maneuvers opportunities
        final DateTimeComponents dtc =
                cycleStart.shiftedBy(firstOffset).getComponents(TimeScalesFactory.getUTC());
        final int startDoy = dtc.getDate().getDayOfYear();
        int doy1 = maneuversDoy[0];
        int doy2 = maneuversDoy[1 % maneuversDoy.length];
        for (int i = 0; i < maneuversDoy.length; ++i) {
            if (startDoy >= maneuversDoy[i]) {
                doy1 = maneuversDoy[(i + 1) % maneuversDoy.length];
                doy2 = maneuversDoy[(i + 2) % maneuversDoy.length];
            }
        }
        while (doy1 <= startDoy) {
            doy1 += 365;
        }
        while (doy2 <= doy1) {
            doy2 += 365;
        }
        AbsoluteDate firstOpportunity  =
                cycleStart.shiftedBy(firstOffset + (doy1 - startDoy) * Constants.JULIAN_DAY);
        AbsoluteDate secondOpportunity =
                firstOpportunity.shiftedBy((doy2 - doy1) * Constants.JULIAN_DAY);

        // check MLST evolution on a very long period (typically much longer than cycle)
        final AbsoluteDate start;
        final AbsoluteDate targetT;
        if (nodeState == null) {
            start   = cycleStart;
            targetT = firstOpportunity.compareTo(cycleEnd) > 0 ? firstOpportunity : secondOpportunity;
        } else {
            start   = nodeState.getDate();
            targetT = secondOpportunity;
        }

        if (iteration == 0) {

            final double mlstCycleStart =
                    mlstModel.mlst(cycleStart, tRef, dhDotDi, mlstRef, iOffsetRef);
            if (mlstCycleStart > getMax() || mlstCycleStart < getMin()) {
                // we are already outside of window, try to get back inside

                // set up a maneuver as soon as possible
                nodeState = findManeuverNode(cycleStart, cycleEnd, propagator);

                // target a centered MLST at cycle end
                targeting = new UnivariateFunction() {
                    /** {@inheritDoc} */
                    public double value(double x) {
                        return mlstModel.mlst(cycleEnd, tRef, dhDotDi, mlstRef, iOffsetRef + scaling * x) -
                               0.5 * (getMin() + getMax());
                    }
                };

            } else {
                // we are within window at start, look for exit conditions

                // look at current excursion (achieved peak, and following tail)
                final UnivariatePointValuePair pvPeak =
                        extremumMLST(start, targetT, mlstModel.increasingInclination());
                final AbsoluteDate peakDate = start.shiftedBy(pvPeak.getPoint());
                final UnivariatePointValuePair pvTail =
                        extremumMLST(peakDate, targetT, !mlstModel.increasingInclination());

                if (pvPeak.getValue() < getMin() || pvPeak.getValue() > getMax()) {
                    // we overshoot the tolerance window on the wrong side (around parabola peak)
                    // set up a maneuver as soon as possible to prevent this early exit
                    nodeState = findManeuverNode(cycleStart, cycleEnd, propagator);
                } else if (pvTail.getValue() < getMin() || pvTail.getValue() > getMax()) {
                    // we exit the tolerance window on the nominal side (at parabola tail)

                    final double mlstExit = pvTail.getValue() < getMin() ? getMin() : getMax();
                    final UnivariateFunction exitFunction = new UnivariateFunction() {
                        /** {@inheritDoc} */
                        public double value(double x) {
                            return mlstModel.mlst(peakDate.shiftedBy(x), tRef, dhDotDi, mlstRef, iOffsetRef) -
                                    mlstExit;
                        }
                    };
                    final double tExit = new BracketingNthOrderBrentSolver(1.0e-10, 5).solve(1000, exitFunction,
                                                                                             0, pvTail.getPoint());
                    if (cycleEnd.durationFrom(peakDate) >= tExit) {
                        // early exit
                        AbsoluteDate searchStart = peakDate.shiftedBy(tExit - 3 * meanPeriod);
                        if (searchStart.durationFrom(cycleStart) < firstOffset) {
                            searchStart = cycleStart.shiftedBy(firstOffset);
                        }
                        nodeState = findManeuverNode(searchStart, cycleEnd, propagator);
                    } else {
                        // late exit
                        final double dt = firstOpportunity.durationFrom(cycleEnd);
                        if (dt > firstOffset) {
                            // we can wait for next cycle to perform the maneuver
                            nodeState = null;
                        } else if (dt < -meanPeriod) {
                            // maneuver opportunity is well inside the cycle, we can find a node close to it
                            nodeState = findManeuverNode(firstOpportunity, cycleEnd, propagator);
                        } else {
                            // maneuver opportunity is too close to cycle end, we need some margin to find the node
                            nodeState = findManeuverNode(cycleEnd.shiftedBy(-3 * meanPeriod), cycleEnd, propagator);
                        }
                    }
                } else {
                    // we stay within bounds for all the monitoring period
                    nodeState = null;
                }

                // target a centered excursion for the observing period
                targeting = new UnivariateFunction() {
                    /** {@inheritDoc} */
                    public double value(double x) {
                        final double savedIOffsetRef = iOffsetRef;
                        iOffsetRef += scaling * x;
                        final UnivariatePointValuePair pvPeak =
                                extremumMLST(start, targetT, mlstModel.increasingInclination());
                        final AbsoluteDate peakDate = start.shiftedBy(pvPeak.getPoint());
                        final UnivariatePointValuePair pvTail =
                                extremumMLST(peakDate, targetT, !mlstModel.increasingInclination());
                        iOffsetRef = savedIOffsetRef;
                        return (pvPeak.getValue() + pvTail.getValue()) - (getMin() + getMax());
                    }
                };

            }

        }

        if (nodeState == null) {
            // no maneuvers needed
            return tunables;
        }

        // compute inclination offset needed to achieve station-keeping target
        double deltaOffset = findZero(targeting, 0, -0.1, 0.1, 1.0e-6, 0.1, 1.0e-10);
        if (Double.isNaN(deltaOffset)) {
            throw new SkatException(SkatMessages.NO_BRACKETING);
        }

        // check for overshoot
        final double savedIOffsetRef = iOffsetRef;
        iOffsetRef += scaling * deltaOffset;
        final UnivariatePointValuePair pvPeak =
                extremumMLST(start, targetT, mlstModel.increasingInclination());
        iOffsetRef = savedIOffsetRef;
        if (pvPeak.getValue() >= getMax() || pvPeak.getValue() <= getMin()) {
            final double safetyMargin = 0.1;
            final double desiredPeak  = pvPeak.getValue() > getMax() ?
                                        ((1 - safetyMargin) * getMax() + safetyMargin * getMin()) :
                                        (safetyMargin * getMax() + (1 - safetyMargin) * getMin());
            deltaOffset = findZero(new UnivariateFunction() {
                /** {@inheritDoc} */
                public double value(double x) {
                    iOffsetRef = savedIOffsetRef + scaling * x;
                    final UnivariatePointValuePair p = extremumMLST(start, targetT, mlstModel.increasingInclination());
                    iOffsetRef = savedIOffsetRef;
                    return p.getValue() - desiredPeak;
                }
            }, deltaOffset,  deltaOffset - 0.1, deltaOffset + 0.1, 1.0e-6, 0.1, 1.0e-10);
            if (Double.isNaN(deltaOffset)) {
                throw new SkatException(SkatMessages.NO_BRACKETING);
            }
        }

        return tuneInclinationManeuver(tunables, reference, nodeState, deltaOffset, compensateLongBurn);

    }

    /** {@inheritDoc} */
    public EventDetector getEventDetector() {
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return null;
    }

}
