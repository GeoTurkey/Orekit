/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.ArrayList;
import java.util.List;

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

    /** Sampled offset to reference points. */
    private final List<Double> sample;

    /** Earth model. */
    private final BodyShape earth;

    /** Reference track points in Earth frame. */
    private final Vector3D[] earthPoints;

    /** Reference track points longitudes. */
    private final double[] longitudes;

    /** Date of first reference point encounter. */
    private AbsoluteDate firstCrossingDate;

    /** Index of first reference point encountered. */
    private int firstCrossingIndex;

    /** Elapsed time between two successive reference points encounters. */
    private double encountersGap;

    /** if true, solar time is computed when crossing the specified latitude from south to north. */
    private boolean ascending;

    /** Simple constructor.
     * <p>
     * The number of days per phasing cycle is approximate in the sense it is
     * expressed neither in solar days nor in sidereal days. It is precisely
     * the number of Earth rotation <em>with respect to the orbital plane</em>,
     * i.e. it takes
     * </p>
     * @param name name of the control law
     * @param scalingDivisor divisor to use for scaling the control law
     * @param controlled name of the controlled spacecraft
     * @param earth Earth model
     * @param latitude reference point latitude
     * @param longitude reference point longitude
     * @param ascending if true, solar time is computed when crossing the
     * specified latitude from south to north
     * @param orbitsPerPhasingCycle number of orbits per phasing cycle
     * @param daysPerPhasingCycle approximate number of days per phasing cycle
     * @param maxLongitude 
     * @param minLongitude 
     * @exception SkatException if orbits and days per phasing cycle are not
     * mutually prime numbers
     */
    public GroundTrackGrid(final String name, final double scalingDivisor,
                           final String controlled, final BodyShape earth,
                           final double latitude, final double longitude, final boolean ascending,
                           final int orbitsPerPhasingCycle, final int daysPerPhasingCycle, 
                           final double minLongitude,
                           final double maxLongitude) throws SkatException {
        super(name, scalingDivisor, controlled, null, 0.0, minLongitude, maxLongitude);
        if (ArithmeticUtils.gcd(orbitsPerPhasingCycle, daysPerPhasingCycle) != 1) {
            throw new SkatException(SkatMessages.PHASING_NUMBERS_NOT_MUTUALLY_PRIMES,
                                    orbitsPerPhasingCycle, daysPerPhasingCycle);
        }

        this.earth = earth;
        this.ascending = ascending;
        // compute the reference longitudes
        longitudes = new double[orbitsPerPhasingCycle];
        earthPoints = new Vector3D[orbitsPerPhasingCycle];
        final double deltaLOneOrbit = (2 * FastMath.PI * daysPerPhasingCycle) / orbitsPerPhasingCycle;
        for (int i = 0; i < orbitsPerPhasingCycle; ++i) {
            longitudes[i] = MathUtils.normalizeAngle(longitude + i * deltaLOneOrbit, FastMath.PI);
            earthPoints[i] = earth.transform(new GeodeticPoint(latitude, longitudes[i], 0));
        }
        sample = new ArrayList<Double>();

    }

    /** {@inheritDoc} */
    @Override
    public void initializeRun(final ScheduledManeuver[] maneuvers,
                              final Propagator propagator,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException {

        super.initializeRun(maneuvers, propagator, start, end);

        // synchronize station-keeping cycle with phasing cycle
        synchronizeCycle(start, propagator);

        // evaluate ground track distances
        // against all reference points encountered during cycle
        final AbsoluteDate endDate = end.shiftedBy(-0.5 * encountersGap);
        final int n = (int) FastMath.floor(endDate.durationFrom(firstCrossingDate) / encountersGap);
        final BracketingNthOrderBrentSolver solver = new BracketingNthOrderBrentSolver(1.0e-3, 5);
        for (int i = 0; i < n; ++i) {

            // find closest approach to ith reference point
            AbsoluteDate date = firstCrossingDate.shiftedBy(i * encountersGap);
            final Vector3D point    = earthPoints[(firstCrossingIndex + i) % earthPoints.length];
            final double adjustment = solver.solve(1000,
                                                   new RadialVelocity(date, point, propagator, earth),
                                                   -encountersGap / 20, encountersGap / 20, 0.0);
            date = date.shiftedBy(adjustment);
            final PVCoordinates closest = propagator.getPVCoordinates(date, earth.getBodyFrame());

            // compute ground track distance
            final GeodeticPoint gp = earth.transform(closest.getPosition(), earth.getBodyFrame(), date);
            final Vector3D subSatellite =
                    earth.transform(new GeodeticPoint(gp.getLatitude(), gp.getLongitude(), 0.0));
            final double distance = subSatellite.distance(point);
            checkLimits(distance);
            // add distance to sample
            sample.add(distance);

        }
             
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
        final double[] data = new double[sample.size()];
        for (int i = 0; i < data.length; ++i) {
            data[i] = sample.get(i);
        }
        return new Percentile().evaluate(data, 75.0);
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
     * @param start start date of the cycle
     * @param propagator propagator valid over the station-keeping cycle
     * @exception OrekitException if orbit cannot be propagated
     */
    private void synchronizeCycle(final AbsoluteDate start, final Propagator propagator)
        throws OrekitException {

        // initialization
        firstCrossingDate  = start;
        firstCrossingIndex = 0;
        double minDistance = Double.POSITIVE_INFINITY;

        // first rough guess, sampling over first orbit using
        // about three points per longitude slot
        // using a few steps margin from beginning to avoid reference
        // points near propagator start boundary
        SpacecraftState initialState = propagator.getInitialState();
        final AbsoluteDate date = (start == null) ? initialState.getDate() : start;
        final double period     = initialState.getKeplerianPeriod();
        final double step       = 100.;/*period / (3 * longitudes.length);*/
        final int safetyMargin  = 2;
        for (int i = safetyMargin; i < 3 * longitudes.length + safetyMargin; ++i) {
            final AbsoluteDate currentDate = date.shiftedBy(i * step);
            PVCoordinates pv = propagator.propagate(currentDate).getPVCoordinates();
            final Vector3D position = pv.getPosition();
            final Vector3D velocity = pv.getVelocity();
            GeodeticPoint p = earth.transform(position, earth.getBodyFrame(), currentDate);
            double deltaL = p.getLatitude() - earth.transform(earthPoints[0], earth.getBodyFrame(), currentDate).getLatitude();
            System.out.println(p + "  " + deltaL);
            // Check if crossing is done in the same sense than the ascending boolean
            final boolean up = velocity.getZ() >= 0. ? true : false;
            if (up == ascending){ 
                System.out.println(Math.toDegrees(p.getLongitude()) + " " + Math.toDegrees(p.getLatitude()));
                System.out.println();
                for (int j = 0; j < earthPoints.length; ++j) {
                    final double distance = position.distance(earthPoints[j]);
                    System.out.println(distance);
                    if (distance < minDistance) {
                        System.out.println("min = " + distance);
                        minDistance        = distance;
                        firstCrossingDate  = currentDate;
                        firstCrossingIndex = j;
                    }            
                }            
            }
        }

        // refine first encounter date
        final BracketingNthOrderBrentSolver solver = new BracketingNthOrderBrentSolver(1.0e-3, 5);
        final double adjustment = solver.solve(1000,
                                               new RadialVelocity(firstCrossingDate,
                                                                  earthPoints[firstCrossingIndex],
                                                                  propagator, earth),
                                               -10*step, 10*step, 0.0);
        firstCrossingDate = firstCrossingDate.shiftedBy(adjustment);

        // find time between first and second reference point encounter
        encountersGap = solver.solve(1000,
                                     new RadialVelocity(firstCrossingDate,
                                                        earthPoints[(firstCrossingIndex + 1) % earthPoints.length],
                                                        propagator, earth),
                                     7 * period / 8, 9 * period / 8, period);

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
