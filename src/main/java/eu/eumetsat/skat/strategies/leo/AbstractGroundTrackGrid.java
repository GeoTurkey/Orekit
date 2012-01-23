/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.apache.commons.math.util.Precision;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.SecularAndHarmonic;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Station-keeping control attempting to follow a specified ground-track grid.
 * <p>
 * The ground-track is defined by a set of latitude/longitude reference points
 * with an associated latitude crossing direction (ascending/descending). The
 * control checks the longitude offset with respect to the reference longitude
 * each time the spacecraft crosses a reference latitude in the specifued
 * direction. Hence the control correspond to an East/West positioning with respect
 * to the grid.
 * </p>
 * <p>
 * The logitude offset evolution is parabolic and is mainly due to atmospheric drag
 * which lower the semi major axis and hence changes orbit period. The initial
 * longitude offset is inherited from the previous cycle motion. The curvature comes
 * from &aring; which depends on current drag. So the only control parameter left for
 * control is the initial longitude drift, which is adjusted by changing the offset
 * [a(t<sub>0</sub>) - a<sub>ref</sub>] at initial time t<sub>0</sub> thanks to
 * in-plane maneuvers. The aim of the control law is to achieve a parabolic motion
 * that remains in the deadband for as long as possible.
 * </p>
 * @author Luc Maisonobe
 */
public abstract class AbstractGroundTrackGrid extends AbstractSKControl {

    /** Threshold under which latitudes are considered equal. */
    private static final double EPSILON_LATITUDE = 1.0e-3;

    /** Limit for separating latitude regions. */
    protected static final double SWITCH_LIMIT = FastMath.toRadians(5.0);

    /** Maximum number of maneuvers to set up in one cycle. */
    protected final int maxManeuvers;

    /** Time offset of the first maneuver with respect to cycle start. */
    protected final double firstOffset;

    /** Minimum time between split parts in number of orbits. */
    protected final int orbitsSeparation;

    /** Earth model. */
    protected final OneAxisEllipsoid earth;

    /** Reference grid points in Earth frame. */
    private final GridPoint[] grid;

    /** Switch latitude to separate extreme latitude points from intermediate ones. */
    private double switchLatitude;

    /** Reference radius of the Earth for the potential model. */
    protected final double referenceRadius;

    /** Central attraction coefficient. */
    protected final double mu;

    /** Un-normalized zonal coefficient. */
    protected final double j2;

    /** Error models for each reference latitude. */
    protected final List<ErrorModel> errorModels;

    /** States at crossing points. */
    protected final List<Crossing> crossings;

    /** State at fit start. */
    protected SpacecraftState fitStart;

    /** Index of the first encountered grid point. */
    protected int firstEncounter;

    /** Reference date for the synchronized grid. */
    protected AbsoluteDate gridReference;

    /** Phasing duration (interval between two grids. */
    protected double phasingDuration;

    /** Iteration number. */
    protected int iteration;

    /** Cycle end. */
    protected AbsoluteDate cycleEnd;

    /** Maneuver-free interval. */
    protected AbsoluteDate[] freeInterval;

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
     * @param referenceRadius reference radius of the Earth for the potential model (m)
     * @param mu central attraction coefficient (m<sup>3</sup>/s<sup>2</sup>)
     * @param j2 un-normalized zonal coefficient (about +1.08e-3 for Earth)
     * @param grid grid points
     * @param maxDistance maximal cross distance to ground track allowed
     * @param horizon time horizon duration
     */
    public AbstractGroundTrackGrid(final String name, final String controlledName, final int controlledIndex,
                           final TunableManeuver model, final double firstOffset,
                           final int maxManeuvers, final int orbitsSeparation,
                           final OneAxisEllipsoid earth,
                           final double referenceRadius, final double mu, final double j2,
                           final List<GridPoint> grid, final double maxDistance, final double horizon)
        throws SkatException {
        super(name, model, controlledName, controlledIndex, null, -1, -maxDistance, maxDistance,
              horizon * Constants.JULIAN_DAY);

        this.firstOffset      = firstOffset;
        this.maxManeuvers     = maxManeuvers;
        this.orbitsSeparation = orbitsSeparation;
        this.earth            = earth;
        this.referenceRadius  = referenceRadius;
        this.mu               = mu;
        this.j2               = j2;
        this.crossings        = new ArrayList<Crossing>();

        // store the grid in chronological order
        this.grid             = grid.toArray(new GridPoint[grid.size()]);
        Arrays.sort(this.grid, new Comparator<GridPoint>() {

            /** {@inheritDoc} */
            public int compare(final GridPoint gp1, final GridPoint gp2) {
                return Double.compare(gp1.getTimeOffset(), gp2.getTimeOffset());
            }

        });

        // extract the limited set of independent point/directions from the grid
        errorModels = new ArrayList<ErrorModel>();
        for (final GridPoint point : this.grid) {
            ErrorModel found = null;
            for (final ErrorModel errorModel : errorModels) {
                if (errorModel.matches(point.getGeodeticPoint().getLatitude(), point.isAscending())) {
                    found = errorModel;
                }
            }
            if (found == null) {
                // this is a new point/direction pair, store it
                errorModels.add(new ErrorModel(point));
            } else {
                // this is an already known point/direction pair
                found.addGridPoint(point);
            }
        }

        // safety checks
        if (errorModels.isEmpty()) {
            throw new SkatException(SkatMessages.NO_GRID_POINTS);
        }
        final ErrorModel m0 = errorModels.get(0);
        for (final ErrorModel errorModel : errorModels) {
            if (errorModel.getCount() != m0.getCount()) {
                throw new SkatException(SkatMessages.GRID_POINTS_NUMBER_MISMATCH,
                                        m0.getCount(), m0.getLatitude(),
                                        errorModel.getCount(), errorModel.getLatitude());
            }
            // rough order of magnitudes values for initialization purposes
            errorModel.resetFitting(AbsoluteDate.J2000_EPOCH, 
                                    0.0, -1.0e-10, -1.0e-17,
                                    1.0e-3, 1.0e-3, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5);
        }

        // phasing is reached one orbit after last
        phasingDuration = (m0.getCount() * m0.getTimeSpan()) / (m0.getCount() - 1);

    }

    /** Initialize one run of the control law.
     * <p>
     * This method is called at the start of each run with a set
     * of parameters. It can be used to reset some internal state
     * in the control law if needed.
     * </p>
     * @param iteration iteration number within the cycle
     * @param maneuvers maneuvers scheduled for this control law
     * @param propagator propagator for the cycle (it already takes
     * the maneuvers into account, but <em>none</em> of the {@link #getEventDetector()
     * event detector} and {@link #getStepHandler() step handler} if any)
     * @param fixedManeuvers list of maneuvers already fixed for the cycle
     * @param start start date of the propagation
     * @param end end date of the propagation
     * @exception OrekitException if something weird occurs with the propagator
     * @exception SkatException if initialization fails
     */
    protected void baseInitializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                                     final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                                     final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException, SkatException {

        resetMarginsChecks();
        this.iteration = iteration;
        this.cycleEnd       = end;

        // limit latitude for switching between latitude and longitude crossings
        double maxLatitude = propagator.getInitialState().getI();
        if (maxLatitude > 0.5 * FastMath.PI) {
            maxLatitude = FastMath.PI - maxLatitude;
        }
        switchLatitude = FastMath.max(maxLatitude - SWITCH_LIMIT, 0);

        double period = phasingDuration / errorModels.get(0).getCount();
        if (iteration == 0) {

            // select a long maneuver-free interval for fitting
            freeInterval = getManeuverFreeInterval(maneuvers, fixedManeuvers,
                                                   start.shiftedBy(period), end.shiftedBy(-period));

            // find the first crossing of one of the grid latitudes
            final double stepSize = period / 100;
            SpacecraftState firstCrossing = null;
            boolean firstAscending = true;
            for (final ErrorModel errorModel : errorModels) {
                if (isIntermediateLatitude(errorModel.getLatitude())) {
                    final SpacecraftState crossing = firstLatitudeCrossing(errorModel.getLatitude(), errorModel.isAscending(),
                                                                           earth, start.shiftedBy(period), end.shiftedBy(-period),
                                                                           stepSize, propagator);
                    if (firstCrossing == null || crossing.getDate().compareTo(firstCrossing.getDate()) < 0) {
                        firstCrossing  = crossing;
                        firstAscending = errorModel.isAscending();
                    }
                }
            }
            if (firstCrossing == null) {
                throw new SkatException(SkatMessages.LATITUDE_NEVER_CROSSED,
                                        FastMath.toDegrees(errorModels.get(0).getLatitude()),
                                        start.shiftedBy(period), end.shiftedBy(-period));
            }

            // identify the grid point corresponding to this first crossing
            final GeodeticPoint gp = earth.transform(firstCrossing.getPVCoordinates().getPosition(),
                                                     firstCrossing.getFrame(), firstCrossing.getDate());
            firstEncounter = -1;
            double minLongitudeGap = Double.POSITIVE_INFINITY;
            for (int i = 0; i < grid.length; ++i) {
                if (Precision.equals(gp.getLatitude(), grid[i].getGeodeticPoint().getLatitude(), EPSILON_LATITUDE) &&
                    (firstAscending == grid[i].isAscending())) {
                    final double longitudeGap = MathUtils.normalizeAngle(gp.getLongitude() -
                                                                         grid[i].getGeodeticPoint().getLongitude(), 0);
                    if (FastMath.abs(longitudeGap) <= minLongitudeGap) {
                        firstEncounter  = i;
                        minLongitudeGap = FastMath.abs(longitudeGap);
                    }
                }
            }
            if (firstEncounter < 0) {
                // this should never happen
                throw new SkatException(SkatMessages.LATITUDE_NEVER_CROSSED,
                                        FastMath.toDegrees(errorModels.get(0).getLatitude()),
                                        start.shiftedBy(period), end.shiftedBy(-period));
            }

            // set up grid reference date
            gridReference = firstCrossing.getDate().shiftedBy(-grid[firstEncounter].getTimeOffset());

        }

        // prepare fitting
        int index = firstEncounter;
        crossings.clear();
        fitStart = null;
        AbsoluteDate date  = gridReference.shiftedBy(grid[index].getTimeOffset());
        while (date != null && date.compareTo(end.shiftedBy(-period)) < 0) {

            int nextIndex = (index + 1) % grid.length;

            // find the reference point crossing for the current grid point
            double gap = grid[nextIndex].getTimeOffset() - grid[index].getTimeOffset();
            if (nextIndex == 0) {
                // wrap point from end of phasing cycle to start of next phasing cycle
                gap += phasingDuration;
            }

            final SpacecraftState crossing;
            if (isIntermediateLatitude(grid[index].getGeodeticPoint().getLatitude())) {
                // intermediate latitude, we find the crossing latitude-wise
                crossing = latitudeCrossing(grid[index].getGeodeticPoint().getLatitude(), earth, date, end,
                                            0.1 * gap, gap, propagator);
            } else {
                crossing = longitudeCrossing(grid[index].getGeodeticPoint().getLongitude(), earth, date, end,
                                             0.1 * gap, gap, propagator);                
            }

            if (crossing != null) {
                if (fitStart == null &&
                    crossing.getDate().compareTo(freeInterval[0]) >= 0 &&
                    crossing.getDate().compareTo(freeInterval[1]) <= 0) {
                    fitStart = crossing;
                }
                crossings.add(new Crossing(crossing, grid[index]));
                index = nextIndex;
                date  = crossing.getDate().shiftedBy(gap);
            } else {
                date = null;
            }

        }

    }

    /** Check if a latitude is an intermediate one or an extreme one.
     * @return true if latitude is an intermediate latitude
     */
    protected boolean isIntermediateLatitude(final double latitude) {
        return latitude >= -switchLatitude && latitude <= switchLatitude;
    }

    /** {@inheritDoc} */
    public EventDetector getEventDetector() {
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return null;
    }

    /** Inner class for grid error models. */
    protected static class ErrorModel extends SecularAndHarmonic {

        /** Medium period model pulsation. */
        private static final double BASE_PULSATION = 2.0 * FastMath.PI / Constants.JULIAN_DAY;

        /** Long period model pulsation. */
        private static final double SUN_PULSATION = 2.0 * FastMath.PI / Constants.JULIAN_YEAR;

        /** Latitude. */
        private final double latitude;

        /** Latitude crossing direction. */
        private final boolean ascending;

        /** Number of grid points this model refers to. */
        private int count;

        /** Time offset of the earliest grid point. */
        private double earliestOffset;

        /** Time offset of the latest grid point. */
        private double latestOffset;

        /** Simple constructor.
         * @param gridPoint grid point
         */
        public ErrorModel(final GridPoint gridPoint) {
            super(2, new double[] {
                SUN_PULSATION, 2 * SUN_PULSATION,
                BASE_PULSATION, 2 * BASE_PULSATION
            });
            this.latitude       = gridPoint.getGeodeticPoint().getLatitude();
            this.ascending      = gridPoint.isAscending();
            this.earliestOffset = gridPoint.getTimeOffset();
            this.latestOffset   = earliestOffset;
            this.count          = 1;
        }

        /** Add one more point in the model.
         * @param grid point to add
         */
        public void addGridPoint(final GridPoint gridPoint) {
            earliestOffset = FastMath.min(earliestOffset, gridPoint.getTimeOffset());
            latestOffset   = FastMath.max(latestOffset,   gridPoint.getTimeOffset());
            ++count;
        }

        /** Get latitude of the model.
         * @return latitude of the model
         */
        public double getLatitude() {
            return latitude;
        }

        /** Get crossing direction.
         * @return true of crossign direction is from South to North
         */
        public boolean isAscending() {
            return ascending;
        }

        /** Get the number of grid points this model refers to.
         * @return number of grid points this model refers to
         */
        public int getCount() {
            return count;
        }

        /** Get the time span between earliest and latest grid points.
         * @return time span between earliest and latest grid points
         */
        public double getTimeSpan() {
            return latestOffset - earliestOffset;
        }

        /** Check if a grid point belongs to this model.
         * @param pointLatitude latitude of the point
         * @param pointAscending latitude crossing direction
         * @return true if grid point has same latitude and crossing direction
         */
        public boolean matches(final double pointLatitude, final boolean pointAscending) {
            return (Precision.equals(pointLatitude, latitude, EPSILON_LATITUDE) &&
                   (pointAscending == ascending));
        }

    }

    /** Inner container for state/grid point pairs at crossings. */
    protected class Crossing {

        /** State. */
        private final SpacecraftState state;

        /** grid point. */
        private final GridPoint gridPoint;

        /** Simple constructor.
         * @param state current state
         * @param gridPoint grid point
         */
        public Crossing(final SpacecraftState state, final GridPoint gridPoint) {
            this.state     = state;
            this.gridPoint = gridPoint;
        }

        /** Get the current state.
         * @return current state
         */
        public SpacecraftState getState() {
            return state;
        }

        /** Get the crossing date.
         * @return crossing date
         */
        public AbsoluteDate getDate() {
            return state.getDate();
        }

        /** Get the grid point.
         * @return grid point
         */
        public GridPoint getGridPoint() {
            return gridPoint;
        }

    }

}
