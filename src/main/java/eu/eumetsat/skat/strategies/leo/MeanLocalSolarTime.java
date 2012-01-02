/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

import org.apache.commons.math.analysis.ParametricUnivariateFunction;
import org.apache.commons.math.analysis.UnivariateFunction;
import org.apache.commons.math.analysis.solvers.BaseUnivariateRealSolver;
import org.apache.commons.math.analysis.solvers.BracketingNthOrderBrentSolver;
import org.apache.commons.math.analysis.solvers.UnivariateRealSolverUtils;
import org.apache.commons.math.exception.NoBracketingException;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.optimization.fitting.CurveFitter;
import org.apache.commons.math.optimization.general.LevenbergMarquardtOptimizer;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
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
import org.orekit.utils.PVCoordinates;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.OrekitWrapperException;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Station-keeping control attempting to get mean local solar time in a deadband.
 * The deadband is located around a main value {@link #center}. A tolerance margin
 * can be define through the minSolarTime and maxSolarTime parameters, defined by
 * construction. If a violation occurs by getting out of the [minSolarTime, maxSolarTime] 
 * interval, a notifier will be triggered and this information will be monitored.
 *  <p>
 * This control value is:
 * <pre> max(|MLST<sub>75</sub> - MLST<sub>c</sub>|,|MLST<sub>c</sub> - MLST<sub>25</sub>|)</pre>
 * where MLST<sub>75</sub> and MLST<sub>25</sub> are the spacecraft mean local solar time 1st and 3rd
 * quartiles evaluated for the complete cycle duration and MLST<sub>c</sub> is the target mean local
 * solar time.
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to MLST<sub>c</sub> attempts to have most of the points mean local solar time
 * for the satellite centered around the target mean local solar time during the
 * station-keeping. 
 * </p>
 * <p>
 * Using quantiles instead of min/max improves robustness with respect to
 * outliers, which occur when starting far from the desired window. Here, we ignore 25%
 * outliers on both sides.
 * </p>
 */
public class MeanLocalSolarTime extends AbstractSKControl {

    /** Medium period model pulsation. */
    private static final double BASE_PULSATION = 2.0 * FastMath.PI / Constants.JULIAN_DAY;

    /** Solar time target */
    private final double center;

    /** In-plane maneuver model. */
    private final TunableManeuver model;

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
    private AbsoluteDate start;

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

    /** Parameters of the fitted semi major axis linear model. */
    private double[] fittedA;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model in-plane maneuver model
     * @param firstOffset time offset of the first maneuver with respect to cycle start
     * @param maxManeuvers maximum number of maneuvers to set up in one cycle
     * @param orbitsSeparation minimum time between split parts in number of orbits
     * @param earth Earth model
     * @param latitude latitude at which solar time should be computed
     * @param ascending if true, solar time is computed when crossing the
     * specified latitude from south to north
     * @param solarTime target solar time ((in fractional hour, i.e 9h30 = 9.5)
     * @param minSolarTime minimum accepted solar time (in fractional hour)
     * @param maxSolarTime maximum accepted solar time (in fractional hour)
     * @exception OrekitException if the UTC-TAI correction cannot be loaded
     */
    public MeanLocalSolarTime(final String name, final String controlledName, final int controlledIndex,
                              final TunableManeuver model, final double firstOffset,
                              final int maxManeuvers, final int orbitsSeparation,
                              final BodyShape earth, final double latitude, final boolean ascending,
                              final double solarTime, final double minSolarTime, final double maxSolarTime)
        throws OrekitException {
        super(name, controlledName, controlledIndex, null, -1, minSolarTime, maxSolarTime);
        this.center               = solarTime;
        this.model                = model;
        this.firstOffset          = firstOffset;
        this.maxManeuvers         = maxManeuvers;
        this.orbitsSeparation     = orbitsSeparation;
        this.latitude             = latitude;
        this.earth                = earth;
        this.ascending            = ascending;
        this.gmod                 = new GMODFrame();

        // rough order of magnitudes values for initialization purposes
        fittedH = new double[] {
            solarTime, -1.0e-10, -1.0e-17, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5
        };
        fittedA = new double[] {
            7.2e6, 0.0, -1.0e-11, 10.0, 10.0, 10.0, 10.0
        };

    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException, SkatException {

        this.iteration = iteration;
        this.start     = start;
        this.end       = end;

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

        // fit parabolic plus medium periods model to mean solar time and semi major axis
        CurveFitter mstFitter = new CurveFitter(new LevenbergMarquardtOptimizer());
        CurveFitter aFitter   = new CurveFitter(new LevenbergMarquardtOptimizer());

        double period = propagator.getInitialState().getKeplerianPeriod();

        // Find the first latitude crossing
        final double stepSize = period / 100;
        SpacecraftState crossing = findFirstCrossing(fitStart.getDate(), end, stepSize, propagator);
        double mst = meanSolarTime(crossing);
        checkMargins(mst);
        double dt = crossing.getDate().durationFrom(fitStart.getDate());
        mstFitter.addObservedPoint(dt, mst);
        aFitter.addObservedPoint(dt, crossing.getA());

        // Find all other latitude crossings from regular schedule
        while (crossing != null && crossing.getDate().shiftedBy(period).compareTo(end) < 0) {

            final AbsoluteDate previous = crossing.getDate();
            crossing = findLatitudeCrossing(previous.shiftedBy(period), end, stepSize,
                                            period / 8, propagator);
            if (crossing != null) {

                // Store current point
                mst = meanSolarTime(crossing);
                checkMargins(mst);
                dt = crossing.getDate().durationFrom(fitStart.getDate());
                mstFitter.addObservedPoint(dt, mst);
                aFitter.addObservedPoint(dt, crossing.getA());

                // use the same time separation to pinpoint next crossing
                period = crossing.getDate().durationFrom(previous);

            }

        }

        fittedH = mstFitter.fit(new ParabolicAndMediumPeriod(), fittedH);
        fittedA = aFitter.fit(new ParabolicAndMediumPeriod(), fittedA);
        System.out.println("# " + fittedA[0] + " + $t * " + fittedA[1] + " + $t * $t * " + fittedA[2] +
                           " + cos(" + BASE_PULSATION + " * $t) * " + fittedA[3] +
                           " + sin(" + BASE_PULSATION + " * $t) * " + fittedA[4] +
                           " + cos(" + (2*BASE_PULSATION) + " * $t) * " + fittedA[5] +
                           " + sin(" + (2*BASE_PULSATION) + " * $t) * " + fittedA[6]);

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
        final double periodicAllocation = 2 * (FastMath.abs(fittedH[3]) + FastMath.abs(fittedH[4]) +
                                               FastMath.abs(fittedH[5]) + FastMath.abs(fittedH[6]));

        // remaining part of the mean local time window for parabolic motion
        final double parabolicAllocation = (getMax() - getMin()) - periodicAllocation;

        // reference semi-major axis corresponding to mean solar time peak
        final double dtPeak = -2 * fittedH[1] / fittedH[2];
        final double aRef   = fittedA[0] + dtPeak * (fittedA[1] + 0.5 * dtPeak * fittedA[2]);

        // maximum cycle duration to fill up the parabolic allocation
        final double maxCycle = FastMath.sqrt(8 * parabolicAllocation / FastMath.abs(fittedH[2]));

        // achieved peak mean local solar time
        final double peakH = fittedH[0] - 0.5 * fittedH[1] * fittedH[1] / fittedH[2];

        if ((fittedH[2] < 0 && peakH < getMin()) || (fittedH[2] > 0 && peakH > getMax())) {
            // the parabolic motion doesn't even enter the window

            // TODO

        } else if ((fittedH[2] < 0 && peakH < getMax()) || (fittedH[2] > 0 && peakH > getMin())) {
            // the parabolic motion peak is within the window (it's the nominal case)

            // TODO

        } else {
            // the parabolic motion goes all the way through the window
            // entering on one side and leaving on the other side, then coming back later

            // TODO

        }

        final ScheduledManeuver[] tuned;
        final ManeuverAdapterPropagator adapterPropagator = new ManeuverAdapterPropagator(reference);
        if (iteration == 0) {
            // we need to first define the number of maneuvers and their initial settings

            // compute the in plane maneuver required to get the initial semi-major axis offset
            // TODO compute the semi major axis offset
            final double deltaA       = Double.NaN;
            final double mu           = fitStart.getMu();
            final double a            = fitStart.getA();
            final double totalDeltaV  = thrustSign() *
                                        FastMath.sqrt(mu * (2 / a - 1 / (a + deltaA))) - FastMath.sqrt(mu / a);

            // compute the number of maneuvers required
            final double limitDV = (totalDeltaV < 0) ? model.getDVInf() : model.getDVSup();
            final int    nMan    = FastMath.min(maxManeuvers,
                                                (int) FastMath.ceil(FastMath.abs(totalDeltaV / limitDV)));
            final double deltaV  = FastMath.max(model.getDVInf(),
                                                FastMath.min(model.getDVSup(), totalDeltaV / nMan));

            tuned = new ScheduledManeuver[tunables.length + nMan];
            System.arraycopy(tunables, 0, tuned, 0, tunables.length);
            changeTrajectory(tuned, 0, tunables.length, adapterPropagator);

            // in order to avoid tempering eccentricity too much,
            // we use a (n+1/2) orbits between maneuvers, where n is an integer
            final double separation = (orbitsSeparation + 0.5) * fitStart.getKeplerianPeriod();

            // add the new maneuvers
            for (int i = 0; i < nMan; ++i) {
                tuned[tunables.length + i] =
                        new ScheduledManeuver(model, fitStart.getDate().shiftedBy(firstOffset + i * separation),
                                              new Vector3D(deltaV, model.getDirection()),
                                              model.getCurrentThrust(), model.getCurrentISP(),
                                              adapterPropagator, false);
            }

        } else {

            // adjust the existing maneuvers

            // compute the in plane maneuver required to get the initial semi-major axis offset
            // TODO compute the semi major axis offset
            final double deltaA       = Double.NaN;
            final double mu           = fitStart.getMu();
            final double a            = fitStart.getA();
            final double deltaVChange = thrustSign() *
                                        FastMath.sqrt(mu * (2 / a - 1 / (a + deltaA))) - FastMath.sqrt(mu / a);

            // distribute the change over all maneuvers
            tuned = tunables.clone();
            changeTrajectory(tuned, 0, tuned.length, adapterPropagator);
            distributeDV(deltaVChange, 0.0, model, tuned, adapterPropagator);

        }

        // finalize propagator
        for (final ScheduledManeuver maneuver : tuned) {
            adapterPropagator.addManeuver(maneuver.getDate(), maneuver.getDeltaV(), maneuver.getIsp());
        }

        return tuned;

    }

    /** Get the sign of the maneuver model with respect to velocity.
     * @return +1 if model thrust is along velocity direction, -1 otherwise
     */
    private double thrustSign() {
        final Vector3D thrustDirection =
                fitStart.getAttitude().getRotation().applyInverseTo(model.getDirection());
        Vector3D velocity = fitStart.getPVCoordinates().getVelocity();
        return FastMath.signum(Vector3D.dotProduct(thrustDirection, velocity));
    }

    /** {@inheritDoc} */
    public EventDetector getEventDetector() {
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return null;
    }

    /** 
     * Find the first crossing of the referene latitude.
     * @param searchStart search start
     * @param end maximal date not to overtake
     * @param stepSize step size to use
     * @param propagator propagator
     * @return first crossing
     * @throws OrekitException if state cannot be propagated
     */
    private SpacecraftState findFirstCrossing(final AbsoluteDate searchStart, final AbsoluteDate end,
                                              final double stepSize, final Propagator propagator)
        throws OrekitException, SkatException {

        double previousLatitude = Double.NaN;
        for (AbsoluteDate date = searchStart; date.compareTo(end) < 0; date = date.shiftedBy(stepSize)) {
            final PVCoordinates pv       = propagator.propagate(date).getPVCoordinates(earth.getBodyFrame());
            final double currentLatitude = earth.transform(pv.getPosition(), earth.getBodyFrame(), date).getLatitude();
            if (((previousLatitude <= latitude) && (currentLatitude >= latitude) &&  ascending) ||
                ((previousLatitude >= latitude) && (currentLatitude <= latitude) && !ascending)) {
                return findLatitudeCrossing(date.shiftedBy(-0.5 * stepSize), end,
                                            0.5 * stepSize, 2 * stepSize, propagator);
            }
            previousLatitude = currentLatitude;
        }

        throw new SkatException(SkatMessages.LATITUDE_NEVER_CROSSED,
                                FastMath.toDegrees(latitude), searchStart, end);

    }
    
    
    /**
     * Find the state at which the reference latitude is crossed.
     * @param guessDate guess date for the crossing
     * @param endDate maximal date not to overtake
     * @param shift shift value used to evaluate the latitude function bracketing around the guess date  
     * @param maxShift maximum value that the shift value can take
     * @param propagator propagator used
     * @return state at latitude crossing time
     * @throws OrekitException if state cannot be propagated
     * @throws NoBracketingException if latitude cannot be bracketed in the search interval
     */
    private SpacecraftState findLatitudeCrossing(final AbsoluteDate guessDate, final AbsoluteDate endDate,
                                                 final double shift, final double maxShift,
                                                 final Propagator propagator)
        throws OrekitException, NoBracketingException {

        try {

            // function evaluating to 0 at latitude crossings
            final UnivariateFunction latitudeFunction = new UnivariateFunction() {
                /** {@inheritDoc} */
                public double value(double x) throws OrekitWrapperException {
                    try {
                        final SpacecraftState state = propagator.propagate(guessDate.shiftedBy(x));
                        final Vector3D position = state.getPVCoordinates(earth.getBodyFrame()).getPosition();
                        final GeodeticPoint point = earth.transform(position, earth.getBodyFrame(), state.getDate());
                        return point.getLatitude() - latitude;
                    } catch (OrekitException oe) {
                        throw new OrekitWrapperException(oe);
                    }
                }
            };

            // try to bracket the encounter
            double span;
            if (guessDate.shiftedBy(shift).compareTo(endDate) > 0) {
                // Take a 1e-3 security margin
                span = endDate.durationFrom(guessDate) - 1e-3;
            } else {
                span = shift;
            }

            while (!UnivariateRealSolverUtils.isBracketing(latitudeFunction, -span, span)) {

                if (2 * span > maxShift) {
                    // let the Apache Commons Math exception be thrown
                    UnivariateRealSolverUtils.verifyBracketing(latitudeFunction, -span, span);
                } else if (guessDate.shiftedBy(2 * span).compareTo(endDate) > 0) {
                    // Out of range :
                    return null;
                }

                // expand the search interval
                span *= 2;

            }

            // find the encounter in the bracketed interval
            final BaseUnivariateRealSolver<UnivariateFunction> solver =
                    new BracketingNthOrderBrentSolver(0.1, 5);
            final double dt = solver.solve(1000, latitudeFunction,-span, span);
            return propagator.propagate(guessDate.shiftedBy(dt));

        } catch (OrekitWrapperException owe) {
            throw owe.getWrappedException();
        }

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
