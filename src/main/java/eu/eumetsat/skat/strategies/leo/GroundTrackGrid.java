/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import org.apache.commons.math.analysis.UnivariateFunction;
import org.apache.commons.math.analysis.solvers.BracketingNthOrderBrentSolver;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.stat.descriptive.rank.Percentile;
import org.apache.commons.math.util.ArithmeticUtils;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinates;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.OrekitWrapperException;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Station-keeping control attempting to follow a specified ground-track
 * at a specified latitude. The ground-track is defined by a latitude and 
 * a longitude. The tolerance deadband is extended from the center value
 * from the minLongitude and maxLongitude parameters defined by construction.
 * If a violation occurs by getting out of the [minLongitude, maxLongitude] 
 * interval, a notifier will be triggered and this information will be monitored.
 * <p>
 * This control value is:
 * <pre>
 *   d<sub>75</sub>
 * </pre>
 * where d<sub>75</sub> is the 75% quantities of ground track distances for
 * all encounters with the reference ground track points for the complete
 * station-keeping cycle duration (which may be completely different from
 * the phasing cycle).
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to 0 attempts to have most of the points close to the reference ground
 * track points.
 * </p>
 * <p>
 * Using quantities instead of min/max improves robustness with respect to
 * outliers, which occur when starting far from the desired window for example
 * at the end of LEOP. Here, we ignore 25% outliers.
 * </p>
 */
public class GroundTrackGrid extends AbstractSKControl {

    /** Achieved value. */
    private double achievedValue;

    /** Earth model. */
    private final BodyShape earth;

    /** Reference track points in Earth frame. */
    private final Vector3D[] earthPoints;

    /** if true, solar time is computed when crossing the specified latitude from south to north. */
    private boolean ascending;

    /** Reference track points latitude. */
    private final double latitude;

    /** Reference track points longitudes. */
    private final double[] longitudes;

    /** Duration of the ignored start part of the cycle. */
    private final double ignoredStartDuration;

    /** Subsampling. */
    private final int subSampling;

    /** Date of first reference point encounter. */
    private AbsoluteDate firstCrossingDate;

    /** Index of first reference point encountered. */
    private int firstCrossingIndex;

    /** Elapsed time between two successive reference points encounters. */
    private double encountersGap;

    /** Propagator. */
    private Propagator propagator;

    /** Cycle start. */
    private AbsoluteDate cycleStart;

    /** cycle end. */
    private AbsoluteDate cycleEnd;

    /** Simple constructor.
     * <p>
     * The number of days per phasing cycle is approximate in the sense it is
     * expressed neither in solar days nor in sidereal days. It is precisely
     * the number of Earth rotation <em>with respect to the orbital plane</em>,
     * i.e. it takes
     * </p>
     * @param name name of the control law
     * @param scalingDivisor divisor to use for scaling the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param earth Earth model
     * @param latitude reference point latitude
     * @param longitude reference point longitude
     * @param ascending if true, solar time is computed when crossing the
     * specified latitude from south to north
     * @param orbitsPerPhasingCycle number of orbits per phasing cycle
     * @param daysPerPhasingCycle approximate number of days per phasing cycle
     * @param maxDistance maximal cross distance to ground track allowed
     * @param ignoredStartDuration duration of the ignored start part of the cycle
     * @param subSampling subsampling number, to speed up computation
     * @exception SkatException if orbits and days per phasing cycle are not
     * mutually prime numbers
     */
    public GroundTrackGrid(final String name, final double scalingDivisor,
                           final String controlledName, final int controlledIndex,
                           final BodyShape earth,
                           final double latitude, final double longitude, final boolean ascending,
                           final int orbitsPerPhasingCycle, final int daysPerPhasingCycle, 
                           final double maxDistance, final double ignoredStartDuration,
                           final int subSampling)
        throws SkatException {
        super(name, scalingDivisor, controlledName, controlledIndex, null, -1,
              0.0, -maxDistance, maxDistance);
        if (ArithmeticUtils.gcd(orbitsPerPhasingCycle, daysPerPhasingCycle) != 1) {
            throw new SkatException(SkatMessages.PHASING_NUMBERS_NOT_MUTUALLY_PRIMES,
                                    orbitsPerPhasingCycle, daysPerPhasingCycle);
        }

        this.earth     = earth;
        this.ascending = ascending;
        this.latitude  = latitude;

        // compute the reference longitudes
        longitudes = new double[orbitsPerPhasingCycle];
        earthPoints = new Vector3D[orbitsPerPhasingCycle];
        final double deltaLOneOrbit = (2 * FastMath.PI * daysPerPhasingCycle) / orbitsPerPhasingCycle;
        for (int i = 0; i < orbitsPerPhasingCycle; ++i) {
            longitudes[i] = MathUtils.normalizeAngle(longitude - i * deltaLOneOrbit, FastMath.PI);
            earthPoints[i] = earth.transform(new GeodeticPoint(latitude, longitudes[i], 0));
        }

        this.ignoredStartDuration = ignoredStartDuration;
        this.subSampling          = subSampling;
        this.achievedValue        = Double.NaN;

    }

    /** {@inheritDoc} */
    @Override
    public void initializeRun(final ScheduledManeuver[] maneuvers,
                              final Propagator propagator,
                              final AbsoluteDate start, final AbsoluteDate end, int rollingCycles) {
        resetLimitsChecks();
        this.achievedValue = Double.NaN;
        this.propagator    = propagator;
        cycleStart         = start;
        cycleEnd           = end;
    }

    /** {@inheritDoc} */
    public double getAchievedValue() throws OrekitException {

        if (Double.isNaN(achievedValue)) {

            // lazy evaluation of the sample
            try {
                // synchronize station-keeping cycle with phasing cycle
                AbsoluteDate searchStart = cycleStart.shiftedBy(ignoredStartDuration);
                if (cycleEnd.durationFrom(searchStart) < propagator.getInitialState().getKeplerianPeriod()) {
                    // when cycles are drastically truncated just before monitoring,
                    // the ignored start may be too much
                    searchStart = cycleStart;
                }
                synchronizeCycle(searchStart, propagator);

                // evaluate ground track distances
                // against reference points encountered during cycle
                final int n = (int) FastMath.floor(cycleEnd.durationFrom(firstCrossingDate) / encountersGap);
                final BracketingNthOrderBrentSolver solver = new BracketingNthOrderBrentSolver(1.0e-2, 5);
                double deltaAdjustment = 0.0;
                double adjustment = 0.0;
                double[] data = new double[(n + subSampling - 2) / subSampling];
                for (int i = 1; i < n - 1; i += subSampling) {

                    // find closest approach to ith reference point
                    AbsoluteDate date = firstCrossingDate.shiftedBy(i * encountersGap);
                    final Vector3D point    = earthPoints[(firstCrossingIndex + i) % earthPoints.length];
                    final double previousAdjustment = adjustment;
                    adjustment = solver.solve(1000,
                                              new RadialVelocity(date, point, propagator, earth),
                                              adjustment + deltaAdjustment - encountersGap / 20,
                                              adjustment + deltaAdjustment + encountersGap / 20,
                                              adjustment + deltaAdjustment);
                    deltaAdjustment = adjustment - previousAdjustment;
                    date = date.shiftedBy(adjustment);
                    final PVCoordinates closest = propagator.getPVCoordinates(date, earth.getBodyFrame());

                    // compute signed ground track distance
                    final GeodeticPoint gp = earth.transform(closest.getPosition(), earth.getBodyFrame(), date);
                    final Vector3D subSatellite =
                            earth.transform(new GeodeticPoint(gp.getLatitude(), gp.getLongitude(), 0.0));
                    double distance = subSatellite.distance(point);
                    if (Vector3D.dotProduct(point.subtract(closest.getPosition()), closest.getMomentum()) < 0) {
                        distance = -distance;
                    }
                    checkLimits(distance);

                    // add distance to sample
                    try {
                    data[(i - 1) / subSampling] = distance;
                    } catch (ArrayIndexOutOfBoundsException ae) {
                        System.out.println("gotcha!");
                    }

                }

                achievedValue = new Percentile().evaluate(data, 75.0);

            } catch (OrekitWrapperException owe) {
                throw owe.getWrappedException();
            }
        }

        return achievedValue;

    }

    /** {@inheritDoc} */
    public EventDetector getEventDetector() {
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return null;
    }

    /** Synchronize station-keeping cycle with phasing cycle.
     * <p>
     * Synchronizing the cycles means finding in the station keeping
     * cycle which reference point (latitude, longitude) is first
     * encountered, when, and how much time separates the next encounter.
     * </p>
     * @param start start date of the search
     * @param propagator propagator valid over the station-keeping cycle
     * @exception OrekitException if orbit cannot be propagated
     */
    private void synchronizeCycle(final AbsoluteDate start, final Propagator propagator)
        throws OrekitException {

        // first rough search based only on latitude
        final Frame earthFrame = earth.getBodyFrame();
        final SpacecraftState initialState = propagator.getInitialState();
        final double period     = initialState.getKeplerianPeriod();
        final double roughStep  = period / 100;
        final int safetyMargin  = 2;
        AbsoluteDate roughDate = start;
        double minDeltaL = Double.POSITIVE_INFINITY;
        for (int i = safetyMargin; i < safetyMargin + 100; ++i) {

            final AbsoluteDate currentDate = start.shiftedBy(i * roughStep);
            final PVCoordinates pv = propagator.propagate(currentDate).getPVCoordinates(earthFrame);

            // Check if crossing is done in the same sense than the ascending boolean
            if (ascending == (pv.getVelocity().getZ() >= 0.0)) {
                final GeodeticPoint gp = earth.transform(pv.getPosition(), earthFrame, currentDate);
                final double deltaL = FastMath.abs(gp.getLatitude() - latitude);
                if (deltaL < minDeltaL) {
                    minDeltaL = deltaL;
                    roughDate = currentDate;
                }
            }

        }

        // improve search, taking longitudes into account
        firstCrossingDate  = start;
        firstCrossingIndex = 0;
        double minDistance = Double.POSITIVE_INFINITY;
        double fineStep = period / (3 * longitudes.length);
        for (double dt = -roughStep; dt < roughStep; dt += fineStep) {
            final AbsoluteDate currentDate = roughDate.shiftedBy(dt);
            final PVCoordinates pv = propagator.propagate(currentDate).getPVCoordinates(earthFrame);
            final GeodeticPoint gp = earth.transform(pv.getPosition(), earthFrame, currentDate);
            final Vector3D subSat  = earth.transform(new GeodeticPoint(gp.getLatitude(),
                                                                       gp.getLongitude(),
                                                                       0.0));
            for (int j = 0; j < earthPoints.length; ++j) {
                final double distance = subSat.distance(earthPoints[j]);
                if (distance < minDistance) {
                    minDistance        = distance;
                    firstCrossingDate  = currentDate;
                    firstCrossingIndex = j;
                }            
            }
        }

        // final refinement
        final BracketingNthOrderBrentSolver solver = new BracketingNthOrderBrentSolver(1.0e-3, 5);
        final double adjustment = solver.solve(1000,
                                               new RadialVelocity(firstCrossingDate,
                                                                  earthPoints[firstCrossingIndex],
                                                                  propagator, earth),
                                               -2 * fineStep, 2 * fineStep, 0.0);
        firstCrossingDate = firstCrossingDate.shiftedBy(adjustment);

        // find time between first and second reference point encounter
        encountersGap = solver.solve(1000,
                                     new RadialVelocity(firstCrossingDate,
                                                        earthPoints[(firstCrossingIndex + 1) % earthPoints.length],
                                                        propagator, earth),
                                     period - roughStep, period + roughStep, period);

    }

    /** Inner function for finding closest approach. */
    private static class RadialVelocity implements UnivariateFunction {

        /** Reference date. */
        private final AbsoluteDate referenceDate;

        /** Reference point. */
        private final Vector3D referencePoint;

        /** Propagator. */
        private final Propagator propagator;

        /** Earth model. */
        private final BodyShape earth;

        /** Simple constructor.
         * @param referenceDate reference date
         * @param referencePoint reference point
         * @param propagator propagator to use
         * @param earth Earth model
         */
        public RadialVelocity(final AbsoluteDate referenceDate,
                              final Vector3D referencePoint,
                              final Propagator propagator,
                              final BodyShape earth) {
            this.referenceDate  = referenceDate;
            this.referencePoint = referencePoint;
            this.propagator     = propagator;
            this.earth          = earth;
        }

        /** {@inheritDoc} */
        public double value(final double x) throws OrekitWrapperException {
            try {
                final AbsoluteDate adjustedDate = referenceDate.shiftedBy(x);
                final PVCoordinates pv =
                        propagator.getPVCoordinates(adjustedDate, earth.getBodyFrame());
                final Vector3D deltaP = pv.getPosition().subtract(referencePoint);
                return deltaP.normalize().dotProduct(pv.getVelocity().normalize());
            } catch (OrekitException oe) {
                throw new OrekitWrapperException(oe);
            }
        }

    }

}
