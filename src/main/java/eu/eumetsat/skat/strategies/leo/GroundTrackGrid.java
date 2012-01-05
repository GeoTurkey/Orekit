/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.apache.commons.math.util.Precision;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
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
 *   (d<sub>25</sub> + d<sub>75</sub>) / 2
 * </pre>
 * where d<sub>25</sub> and d<sub>75</sub> are respectively the 25% quantile and
 * the 75% quantile of signed ground track distances for the considered
 * encounters with the reference ground track points for the complete
 * station-keeping cycle duration (which may be completely different from
 * the phasing cycle).
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to 0 attempts to have most of the points close to the reference ground
 * track points.
 * </p>
 * <p>
 * Using quantiles instead of min/max improves robustness with respect to
 * outliers, which occur when starting far from the desired window for example
 * at the end of LEOP. Here, we ignore 25% outliers.
 * </p>
 */
public class GroundTrackGrid extends AbstractSKControl {

    /** Threshold under which latitudes are considered equal. */
    private static final double EPSILON_LATITUDE = 1.0e-3;

    /** Maximum number of maneuvers to set up in one cycle. */
    private final int maxManeuvers;

    /** Time offset of the first maneuver with respect to cycle start. */
    private final double firstOffset;

    /** Minimum time between split parts in number of orbits. */
    private final int orbitsSeparation;

    /** Earth model. */
    private final BodyShape earth;

    /** Reference grid points in Earth frame. */
    private final GridPoint[] grid;

    /** Excerpt from the grid with only one point per latitude/crossing direction. */
    private final List<GridPoint> gridExcerpt;

    /** Index of the first encountered grid point. */
    private int firstEncounter;

    /** Reference date for the synchronized grid. */
    private AbsoluteDate gridReference;

    /** Phasing duration (interval between two grids. */
    private double phasingDuration;

    /** Simple constructor.
     * <p>
     * The number of days per phasing cycle is approximate in the sense it is
     * expressed neither in solar days nor in sidereal days. It is precisely
     * the number of Earth rotation <em>with respect to the orbital plane</em>.
     * </p>
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model out-of-plane maneuver model
     * @param firstOffset time offset of the first maneuver with respect to cycle start
     * @param maxManeuvers maximum number of maneuvers to set up in one cycle
     * @param orbitsSeparation minimum time between split parts in number of orbits
     * @param earth Earth model
     * @param grid grid points
     * @param maxDistance maximal cross distance to ground track allowed
     */
    public GroundTrackGrid(final String name, final String controlledName, final int controlledIndex,
                           final TunableManeuver model, final double firstOffset,
                           final int maxManeuvers, final int orbitsSeparation,
                           final BodyShape earth, final List<GridPoint> grid, final double maxDistance)
        throws SkatException {
        super(name, model, controlledName, controlledIndex, null, -1, -maxDistance, maxDistance);

        this.firstOffset      = firstOffset;
        this.maxManeuvers     = maxManeuvers;
        this.orbitsSeparation = orbitsSeparation;
        this.earth            = earth;

        // store the grid in chronological order
        this.grid             = grid.toArray(new GridPoint[grid.size()]);
        Arrays.sort(this.grid, new Comparator<GridPoint>() {

            /** {@inheritDoc} */
            public int compare(final GridPoint gp1, final GridPoint gp2) {
                return Double.compare(gp1.getTimeOffset(), gp2.getTimeOffset());
            }

        });

        // extract the limited set of independent latitudes/directions from the grid
        gridExcerpt = new ArrayList<GridPoint>();
        for (final GridPoint regularPoint : this.grid) {
            boolean found = false;
            for (final GridPoint excerptPoint : gridExcerpt) {
                if (Precision.equals(excerptPoint.getLatitude(), regularPoint.getLatitude(), EPSILON_LATITUDE) &&
                    (excerptPoint.isAscending() == regularPoint.isAscending())) {
                    found = true;
                }
            }
            if (!found) {
                // this is a new latitude/direction pair, store it
                gridExcerpt.add(regularPoint);
            }
        }

    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException, SkatException {

        resetMarginsChecks();

        if (iteration == 0) {

            // select a long maneuver-free interval for fitting
            final AbsoluteDate[] freeInterval = getManeuverFreeInterval(maneuvers, fixedManeuvers, start, end);

            // find the first crossing of one of the grid latitudes
            final double period = propagator.getInitialState().getKeplerianPeriod();
            final double stepSize = period / 100;
            SpacecraftState firstCrossing = null;
            boolean firstAscending = true;
            for (final GridPoint point : gridExcerpt) {
                final SpacecraftState crossing = findFirstCrossing(point.getLatitude(), point.isAscending(),
                                                                   earth, freeInterval[0], freeInterval[1],
                                                                   stepSize, propagator);
                if (firstCrossing == null || crossing.getDate().compareTo(firstCrossing.getDate()) < 0) {
                    firstCrossing  = crossing;
                    firstAscending = point.isAscending();
                }
            }
            if (firstCrossing == null) {
                throw new SkatException(SkatMessages.LATITUDE_NEVER_CROSSED,
                                        FastMath.toDegrees(gridExcerpt.get(0).getLatitude()),
                                        freeInterval[0], freeInterval[1]);
            }

            // identify the grid point corresponding to this first crossing
            final GeodeticPoint gp = earth.transform(firstCrossing.getPVCoordinates().getPosition(),
                                                     firstCrossing.getFrame(), firstCrossing.getDate());
            firstEncounter = -1;
            double minLongitudeGap = Double.POSITIVE_INFINITY;
            for (int i = 0; i < grid.length; ++i) {
                if (Precision.equals(gp.getLatitude(), grid[i].getLatitude(), EPSILON_LATITUDE) &&
                    (firstAscending == grid[i].isAscending())) {
                    final double longitudeGap = MathUtils.normalizeAngle(gp.getLongitude() - grid[i].getLongitude(), 0);
                    if (FastMath.abs(longitudeGap) <= minLongitudeGap) {
                        firstEncounter  = i;
                        minLongitudeGap = FastMath.abs(longitudeGap);
                    }
                }
            }
            if (firstEncounter < 0) {
                // this should never happen
                throw new SkatException(SkatMessages.LATITUDE_NEVER_CROSSED,
                                        FastMath.toDegrees(gridExcerpt.get(0).getLatitude()),
                                        freeInterval[0], freeInterval[1]);
            }

            // set up grid reference date
            gridReference = firstCrossing.getDate().shiftedBy(-grid[firstEncounter].getTimeOffset());

            // find cycle phasing duration
            phasingDuration = Double.POSITIVE_INFINITY;
            final AbsoluteDate lastGridCrossing = gridReference.shiftedBy(grid[grid.length - 1].getTimeOffset());
            if (freeInterval[1].compareTo(lastGridCrossing) > 0) {
                // the free interval goes past the last grid point,
                // we must find the duration to add to grid reference when wrapping around grid
                final SpacecraftState wrappedCrossing =
                        findFirstCrossing(grid[0].getLatitude(), grid[0].isAscending(),
                                          earth, lastGridCrossing, freeInterval[1],
                                          stepSize, propagator);
                if (wrappedCrossing != null) {
                    final AbsoluteDate nextGridReference =
                            wrappedCrossing.getDate().shiftedBy(-grid[0].getTimeOffset());
                    phasingDuration = nextGridReference.durationFrom(gridReference);
                }

            }

        }

    }

    /** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException {
        // TODO
        throw SkatException.createInternalError(null);
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
