/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.apache.commons.math.util.Precision;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.AdapterPropagator;
import org.orekit.propagation.analytical.J2DifferentialEffect;
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
public class GroundTrackGrid extends AbstractSKControl {

    /** Threshold under which latitudes are considered equal. */
    private static final double EPSILON_LATITUDE = 1.0e-3;

    /** Limit for switching between latitude/longitude crossings. */
    private static final double SWITCH_LIMIT = FastMath.toRadians(5.0);

    /** Maximum number of maneuvers to set up in one cycle. */
    private final int maxManeuvers;

    /** Time offset of the first maneuver with respect to cycle start. */
    private final double firstOffset;

    /** Minimum time between split parts in number of orbits. */
    private final int orbitsSeparation;

    /** Earth model. */
    private final OneAxisEllipsoid earth;

    /** Reference radius of the Earth for the potential model. */
    private final double referenceRadius;

    /** Central attraction coefficient. */
    private final double mu;

    /** Un-normalized zonal coefficient. */
    private final double j2;

    /** Reference grid points in Earth frame. */
    private final GridPoint[] grid;

    /** Error models for each reference latitude. */
    private final List<ErrorModel> errorModels;

    /** State at fit start. */
    private SpacecraftState fitStart;

    /** Mean fitting parameters for longitude error. */
    private final double[] fittedDL;

    /** Index of the first encountered grid point. */
    private int firstEncounter;

    /** Reference date for the synchronized grid. */
    private AbsoluteDate gridReference;

    /** Phasing duration (interval between two grids. */
    private double phasingDuration;

    /** Iteration number. */
    private int iteration;

    /** Cycle end. */
    private AbsoluteDate cycleEnd;

    /** Safety margin on longitude error window. */
    private final double safetyMargin;

    /** Indicator for side targeting. */
    private boolean forceReset;

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
    public GroundTrackGrid(final String name, final String controlledName, final int controlledIndex,
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
        this.fittedDL         = new double[3];
        this.safetyMargin     = 0.1 * maxDistance;

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
                if (errorModel.matches(point.getLatitude(), point.isAscending())) {
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
        }

        // phasing is reached one orbit after last
        phasingDuration = (m0.getCount() * m0.getTimeSpan()) / (m0.getCount() - 1);

    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
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
        final double switchLatitude = FastMath.max(maxLatitude - SWITCH_LIMIT, 0);

        if (iteration == 0) {

            forceReset = false;

            // select a long maneuver-free interval for fitting
            double period = phasingDuration / errorModels.get(0).getCount();
            final AbsoluteDate[] freeInterval = getManeuverFreeInterval(maneuvers, fixedManeuvers,
                                                                        start.shiftedBy(period),
                                                                        end.shiftedBy(-period));
            if (freeInterval[1].durationFrom(freeInterval[0]) < period) {
                // if cycle is too short, assume limits are not violated
                checkMargins(freeInterval[0], 0.5 * (getMin() + getMax()));
                return;
            }

            // find the first crossing of one of the grid latitudes
            final double stepSize = period / 100;
            SpacecraftState firstCrossing = null;
            boolean firstAscending = true;
            for (final ErrorModel errorModel : errorModels) {
                if (errorModel.getLatitude() >= -switchLatitude && errorModel.getLatitude() <= switchLatitude) {
                    final SpacecraftState crossing = firstLatitudeCrossing(errorModel.getLatitude(), errorModel.isAscending(),
                                                                           earth, freeInterval[0], freeInterval[1],
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
                                        FastMath.toDegrees(errorModels.get(0).getLatitude()),
                                        freeInterval[0], freeInterval[1]);
            }

            // set up grid reference date
            gridReference = firstCrossing.getDate().shiftedBy(-grid[firstEncounter].getTimeOffset());

        }

        // fit longitude offsets to parabolic models
        int index = firstEncounter;
        fitStart = propagator.propagate(gridReference.shiftedBy(grid[index].getTimeOffset()));
        for (final ErrorModel errorModel : errorModels) {
            errorModel.resetFitting(fitStart.getDate(), fittedDL);
        }
        AbsoluteDate date  = fitStart.getDate();
        while (date != null && date.compareTo(end) < 0) {

            int nextIndex = (index + 1) % grid.length;

            // find the reference point crossing for the current grid point
            double gap = grid[nextIndex].getTimeOffset() - grid[index].getTimeOffset();
            if (nextIndex == 0) {
                // wrap point from end of phasing cycle to start of next phasing cycle
                gap += phasingDuration;
            }
            final SpacecraftState crossing;
            if (grid[index].getLatitude() >= -switchLatitude && grid[index].getLatitude() <= switchLatitude) {
                // intermediate latitude, we find the crossing latitude-wise
                crossing = latitudeCrossing(grid[index].getLatitude(), earth, date, end, 0.1 * gap, gap, propagator);
                if (crossing != null) {
                    Vector3D position = crossing.getPVCoordinates(earth.getBodyFrame()).getPosition();
                    final GeodeticPoint gp = earth.transform(position, earth.getBodyFrame(), crossing.getDate());

                    double dl = MathUtils.normalizeAngle(gp.getLongitude() - grid[index].getLongitude(), 0);
                    checkMargins(crossing.getDate(), dl * FastMath.hypot(position.getX(), position.getY()));

                    for (final ErrorModel errorModel : errorModels) {
                        if (errorModel.matches(grid[index].getLatitude(), grid[index].isAscending())) {
                            errorModel.addPoint(crossing.getDate(), dl);
                        }
                    }

                    // go to next grid point
                    index = nextIndex;
                    date  = crossing.getDate().shiftedBy(gap);

                } else {
                    date = null;
                }
            } else {
                // extreme latitude, we cannot use this point for in-plane control

                // go to next grid point
                index = nextIndex;
                date  = date.shiftedBy(gap);

            }

        }

        // fit all point/direction specific models and compute a mean model
        Arrays.fill(fittedDL, 0);
        for (final ErrorModel errorModel : errorModels) {
            errorModel.fit();
            final double[] f = errorModel.getFittedParameters();
            for (int i = 0; i < fittedDL.length; ++i) {
                fittedDL[i] += f[i];
            }
        }
        for (int i = 0; i < fittedDL.length; ++i) {
            fittedDL[i] /= errorModels.size();
        }

    }

    /** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException {

        final double dlMax    = getMax() / earth.getEquatorialRadius();
        final double dlSafety = (getMax() - safetyMargin) / earth.getEquatorialRadius();

        // which depends on current state
        final double newDLdot;
        if (forceReset || (fittedDL[2] < 0 && fittedDL[0] > dlMax) || (fittedDL[2] > 0 && fittedDL[0] < -dlMax)) {
            // the start point is already on the wrong side of the window

            // make sure once we select this option, we stick to it for all iterations
            forceReset = true;

            // the current cycle is already bad, we set up a target to start a new cycle
            // at time horizon with good initial conditions, and reach this target by changing dlDot(t0)
            final double targetT  = cycleEnd.durationFrom(fitStart.getDate()) + firstOffset;
            final double targetDL = FastMath.copySign(dlMax, fittedDL[2]);
            newDLdot = (targetDL - fittedDL[0]) / targetT - fittedDL[2] * targetT;

        } else {
            // the start point is on the right side of the window

            final double tPeak   = -0.5 * fittedDL[1] / fittedDL[2];
            final double dlPeak  = fittedDL[0] + 0.5 * fittedDL[1] * tPeak;
            final double finalT  = cycleEnd.durationFrom(fitStart.getDate()) + firstOffset;
            final double finalDL = fittedDL[0] + finalT * (fittedDL[1] + finalT * fittedDL[2]);

            final boolean intermediateExit = tPeak > 0 && tPeak < finalT && FastMath.abs(dlPeak) > dlMax;
            final boolean finalExit        = FastMath.abs(finalDL) >= dlMax;
            if (intermediateExit || finalExit) {
                // longitude error exits the window limit before next cycle

                // we target a future longitude error peak osculating window boundary
                // (taking a safety margin into account if possible)
                double targetDL = FastMath.copySign(FastMath.abs(fittedDL[0]) >= dlSafety ? dlMax : dlSafety,
                                                    -fittedDL[2]);
                newDLdot = FastMath.copySign(FastMath.sqrt(4 * fittedDL[2] * (fittedDL[0] - targetDL)),
                                             (tPeak > 0) ? fittedDL[1] : -fittedDL[1]);

            } else {
                // longitude error stays within bounds up to next cycle,
                // we don't change anything on the maneuvers
                return tunables;
            }

        }

        // compute semi major axis offset needed to achieve station-keeping target
        final double dlDotDa = -3 * FastMath.PI / (fitStart.getA() * Constants.JULIAN_DAY);
        final double deltaA  = (newDLdot - fittedDL[1]) / dlDotDa;

        final ScheduledManeuver[] tuned;
        final AdapterPropagator adapterPropagator = new AdapterPropagator(reference);
        if (iteration == 0) {
            // we need to first define the number of maneuvers and their initial settings

            // compute the in plane maneuver required to get this initial semi-major axis offset
            final double mu           = fitStart.getMu();
            final double a            = fitStart.getA();
            final double totalDeltaV  = thrustSignVelocity(fitStart) *
                                        FastMath.sqrt(mu * (2 / a - 1 / (a + deltaA))) - FastMath.sqrt(mu / a);

            // in order to avoid tempering eccentricity too much,
            // we use a (n+1/2) orbits between maneuvers, where n is an integer
            final double separation = (orbitsSeparation + 0.5) * fitStart.getKeplerianPeriod();

            // compute the number of maneuvers required
            final TunableManeuver model = getModel();
            final double limitDV = (totalDeltaV < 0) ? model.getDVInf() : model.getDVSup();
            int nMan = FastMath.min(maxManeuvers, (int) FastMath.ceil(FastMath.abs(totalDeltaV / limitDV)));
            while (fitStart.getDate().shiftedBy(firstOffset + (nMan - 1) * separation).getDate().compareTo(cycleEnd) >= 0) {
                --nMan;
            }
            final double deltaV = FastMath.max(model.getDVInf(), FastMath.min(model.getDVSup(), totalDeltaV / nMan));

            tuned = new ScheduledManeuver[tunables.length + nMan];
            System.arraycopy(tunables, 0, tuned, 0, tunables.length);
            changeTrajectory(tuned, 0, tunables.length, adapterPropagator);

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

            // compute the in plane maneuver required to get this initial semi-major axis offset
            final double mu           = fitStart.getMu();
            final double a            = fitStart.getA();
            final double deltaVChange = thrustSignVelocity(fitStart) *
                                        FastMath.sqrt(mu * (2 / a - 1 / (a + deltaA))) - FastMath.sqrt(mu / a);

            // distribute the change over all maneuvers
            tuned = tunables.clone();
            changeTrajectory(tuned, 0, tuned.length, adapterPropagator);
            distributeDV(deltaVChange, 0.0, tuned, adapterPropagator);

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

    /** Inner class for longitude error models. */
    private static class ErrorModel extends SecularAndHarmonic {

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
            super(2, new double[0]);
            this.latitude       = gridPoint.getLatitude();
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

}
