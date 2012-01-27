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
import org.orekit.frames.Transform;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;
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

    /** Error models for longitude at each reference latitude. */
    protected final List<ErrorModel> longitudeModels;

    /** Error models for inclination at each reference latitude. */
    protected final List<ErrorModel> inclinationModels;

    /** States at crossing points. */
    protected final List<Crossing> crossings;

    /** State at fit start. */
    protected SpacecraftState fitStart;

    /** Index of the first encountered grid point. */
    protected int firstEncounter;

    /** Reference date for the synchronized grid. */
    protected AbsoluteDate gridReference;

    /** Phasing duration (interval between two grids. */
    protected final double phasingDuration;

    /** Mean period. */
    protected final double meanPeriod;

    /** Iteration number. */
    protected int iteration;

    /** Cycle end. */
    protected AbsoluteDate cycleEnd;

    /** Maneuver-free interval. */
    protected AbsoluteDate[] freeInterval;

    /** Mean fitting parameters for ascending node error. */
    protected final double[] fittedDN;

    /** Mean fitting parameters for inclination error. */
    protected final double[] fittedDI;

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
        this.fittedDN         = new double[3];
        this.fittedDI         = new double[2];

        // store the grid in chronological order
        this.grid             = grid.toArray(new GridPoint[grid.size()]);
        Arrays.sort(this.grid, new Comparator<GridPoint>() {

            /** {@inheritDoc} */
            public int compare(final GridPoint gp1, final GridPoint gp2) {
                return Double.compare(gp1.getTimeOffset(), gp2.getTimeOffset());
            }

        });

        // extract the limited set of independent point/directions from the grid
        longitudeModels   = createErrorModels(true, false);
        inclinationModels = createErrorModels(false, true);

        // phasing is reached one orbit after last
        final ErrorModel m0 = longitudeModels.get(0);
        phasingDuration = (m0.getCount() * m0.getTimeSpan()) / (m0.getCount() - 1);
        meanPeriod      = phasingDuration / m0.getCount();

    }

    /** Create a list of error models.
     * @param ignoreExtremeLatitude if true, points close to extreme latitude should be avoided
     * @param ignoreEquatorialLatitude if true, points close to equatorial latitude should be avoided
     * @return list of error models
     * @exception SkatException if there are no grid points
     */
    private List<ErrorModel> createErrorModels(final boolean ignoreExtremeLatitude,
                                               final boolean ignoreEquatorialLatitude)
        throws SkatException {

        List<ErrorModel> models   = new ArrayList<ErrorModel>();

        for (final GridPoint point : grid) {
            final boolean avoidedExtreme    = isExtremeLatitude(point.getGeodeticPoint().getLatitude()) &&
                                              ignoreExtremeLatitude;
            final boolean avoidedEquatorial = isEquatorialLatitude(point.getGeodeticPoint().getLatitude()) &&
                                              ignoreEquatorialLatitude;
            if (!(avoidedExtreme || avoidedEquatorial)) {
                ErrorModel found = null;
                for (final ErrorModel errorModel : models) {
                    if (errorModel.matches(point.getGeodeticPoint().getLatitude(), point.isAscending())) {
                        found = errorModel;
                    }
                }
                if (found == null) {
                    // this is a new point/direction pair, store it
                    models.add(new ErrorModel(point));
                } else {
                    // this is an already known point/direction pair
                    found.addGridPoint(point);
                }
            }
        }

        // safety checks
        if (models.isEmpty()) {
            throw new SkatException(SkatMessages.NO_GRID_POINTS);
        }
        final ErrorModel m0 = models.get(0);
        for (final ErrorModel errorModel : models) {
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

        return models;

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
        this.cycleEnd  = end;

        // limit latitude for switching between latitude and longitude crossings
        double maxLatitude = propagator.getInitialState().getI();
        if (maxLatitude > 0.5 * FastMath.PI) {
            maxLatitude = FastMath.PI - maxLatitude;
        }
        switchLatitude = FastMath.max(maxLatitude - SWITCH_LIMIT, 0);

        if (iteration == 0) {

            // select a long maneuver-free interval for fitting
            freeInterval = getManeuverFreeInterval(maneuvers, fixedManeuvers,
                                                   start.shiftedBy(meanPeriod), end.shiftedBy(-meanPeriod));

            // find the first crossing of one of the grid latitudes
            final double stepSize = meanPeriod / 100;
            SpacecraftState firstCrossing = null;
            boolean firstAscending = true;
            for (final ErrorModel errorModel : longitudeModels) {
                final SpacecraftState crossing = firstLatitudeCrossing(errorModel.getLatitude(), errorModel.isAscending(),
                                                                       earth, start.shiftedBy(meanPeriod), end.shiftedBy(-meanPeriod),
                                                                       stepSize, propagator);
                if (firstCrossing == null || crossing.getDate().compareTo(firstCrossing.getDate()) < 0) {
                    firstCrossing  = crossing;
                    firstAscending = errorModel.isAscending();
                }
            }
            if (firstCrossing == null) {
                throw new SkatException(SkatMessages.LATITUDE_NEVER_CROSSED,
                                        FastMath.toDegrees(longitudeModels.get(0).getLatitude()),
                                        start.shiftedBy(meanPeriod), end.shiftedBy(-meanPeriod));
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
                                        FastMath.toDegrees(longitudeModels.get(0).getLatitude()),
                                        start.shiftedBy(meanPeriod), end.shiftedBy(-meanPeriod));
            }

            // set up grid reference date
            gridReference = firstCrossing.getDate().shiftedBy(-grid[firstEncounter].getTimeOffset());

        }

        // prepare fitting
        int index = firstEncounter;
        crossings.clear();
        fitStart = null;
        AbsoluteDate date  = gridReference.shiftedBy(grid[index].getTimeOffset());
        while (date != null && date.compareTo(end.shiftedBy(-meanPeriod)) < 0) {

            int nextIndex = (index + 1) % grid.length;

            // find the reference point crossing for the current grid point
            double gap = grid[nextIndex].getTimeOffset() - grid[index].getTimeOffset();
            if (nextIndex == 0) {
                // wrap point from end of phasing cycle to start of next phasing cycle
                gap += phasingDuration;
            }

            final SpacecraftState crossing;
            if (isExtremeLatitude(grid[index].getGeodeticPoint().getLatitude())) {
                // extreme latitude, we find the crossing longitude-wise
                crossing = longitudeCrossing(grid[index].getGeodeticPoint().getLongitude(), earth, date, end,
                                             0.1 * gap, gap, propagator);
            } else {
                // intermediate latitude, we find the crossing latitude-wise
                crossing = latitudeCrossing(grid[index].getGeodeticPoint().getLatitude(), earth, date, end,
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

        if (fitStart != null) {

            // fit longitude and inclination offsets, first pass
            Arrays.fill(fittedDN, 0.0);
            Arrays.fill(fittedDI, 0.0);
            fitNodeOffsets();
            fitInclinationOffsets();

            // fit longitude and inclination offsets, second pass, using the coupling between inclination and node fits
            fitNodeOffsets();
            fitInclinationOffsets();

            // preserve fitting history for loop detection purposes
            if (iteration == 0) {
                clearHistory();
            }
            addQuadraticFit(fittedDN, 0, freeInterval[1].durationFrom(fitStart.getDate()));

        }

        // check the margins
        for (final Crossing crossing : crossings) {
            final SpacecraftState state = crossing.getState();
            final PVCoordinates current = state.getPVCoordinates();
            final Transform t           = state.getFrame().getTransformTo(earth.getBodyFrame(), state.getDate());
            final GeodeticPoint gp      = earth.transform(t.transformPosition(current.getPosition()), earth.getBodyFrame(), state.getDate());
            final Vector3D subSat       = earth.transform(new GeodeticPoint(gp.getLatitude(), gp.getLongitude(), 0.0));
            final Vector3D delta        = subSat.subtract(crossing.getGridPoint().getCartesianPoint());
            final double dot            = Vector3D.dotProduct(delta, t.transformVector(current.getMomentum()));
            checkMargins(state.getDate(), FastMath.copySign(delta.getNorm(), dot));
        }

    }

    /** Fit longitude offsets.
     * @exception OrekitException if states cannot be converted to Earth frame
     */
    private void fitNodeOffsets()
        throws OrekitException {

        // fit longitude offsets to parabolic models
        for (final ErrorModel errorModel : longitudeModels) {
            errorModel.resetFitting(fitStart.getDate(), errorModel.getFittedParameters());
        }

        final double referenceI = fitStart.getI();

        for (final Crossing crossing : crossings) {
            if (!isExtremeLatitude(crossing.getGridPoint().getGeodeticPoint().getLatitude())) {
                // non-extreme latitude point, we can use its longitude as an indicator for offset

                // ascending node for current point
                final SpacecraftState state = crossing.getState();
                final Vector3D current      = state.getPVCoordinates().getPosition();
                final double dt             = crossing.getDate().durationFrom(fitStart.getDate());
                final double deltaI         = fittedDI[0] + dt * fittedDI[1];
                final double currentNode    = computeNode(current.normalize(), referenceI + deltaI,
                                                          crossing.getGridPoint().isAscending());

                // ascending node for reference grid point
                final Transform transform   = earth.getBodyFrame().getTransformTo(state.getFrame(), state.getDate());
                final Vector3D reference    = transform.transformPosition(crossing.getGridPoint().getCartesianPoint());
                final double referenceNode  = computeNode(reference.normalize(), referenceI,
                                                          crossing.getGridPoint().isAscending());

                final double deltaN = MathUtils.normalizeAngle(currentNode - referenceNode, 0.0);

                if (state.getDate().compareTo(freeInterval[0]) >= 0 &&
                    state.getDate().compareTo(freeInterval[1]) <= 0) {
                    for (final ErrorModel errorModel : longitudeModels) {
                        if (errorModel.matches(crossing.getGridPoint().getGeodeticPoint().getLatitude(),
                                               crossing.getGridPoint().isAscending())) {
                            errorModel.addPoint(crossing.getDate(), deltaN);
                        }
                    }
                }

            }
        }

        // fit all point/direction specific models and compute a mean model
        Arrays.fill(fittedDN, 0);
        for (final ErrorModel errorModel : longitudeModels) {
            errorModel.fit();
            final double[] f = errorModel.approximateAsPolynomialOnly(2, fitStart.getDate(), 2, 2,
                                                                      fitStart.getDate(), freeInterval[1],
                                                                      meanPeriod);
            for (int i = 0; i < fittedDN.length; ++i) {
                fittedDN[i] += f[i];
            }
        }
        for (int i = 0; i < fittedDN.length; ++i) {
            fittedDN[i] /= longitudeModels.size();
        }

    }

    /** Compute right ascension of ascending node.
     * <p>
     * The direction cosines or spacecraft in an inertial frames can be computed as:
     * <pre>
     *  x = cos(Omega) cos(alpha) - sin(Omega) cos(i) sin(alpha)
     *  y = sin(Omega) cos(alpha) + cos(Omega) cos(i) sin(alpha)
     *  z = sin(i) sin(alpha)
     * </pre>
     * so if we know these direction cosines (x, y, z) and inclination i, we
     * can deduce alpha from third equation (z) :
     * sin(alpha) = z/sin(i), cos(alpha) = sign(zDot) * sqrt[1 - sin^2(alpha)]
     * once we know cos(alpha) and sin(alpha), we can compute:
     * <pre>
     * cos(alpha) x + cos(i) sin(alpha) y = cos(Omega) [cos^2(alpha) + cos^2(i) sin^2(alpha)]
     * cos(alpha) y - cos(i) sin(alpha) x = sin(Omega) [cos^2(alpha) + cos^2(i) sin^2(alpha)]
     * </pre>
     * hence we deduce Omega from (x, y, z) and an assumed value for i
     *</p>
     * @param direction normalized point direction in inertial frame
     * @param i assumed inclination
     * @param ascending if true, spacecraft is moving in the South/North direction
     * @return right ascension of ascending node
     */
    private double computeNode(final Vector3D direction, final double i, final boolean ascending) {

        final double cosI = FastMath.cos(i);
        final double sinI = FastMath.sin(i);

        // compute true latitude argument trigonometric functions
        final double sinAlpha = direction.getZ() / sinI;
        final double cosAlpha  = (ascending ? +1 : -1) * FastMath.sqrt(1 - sinAlpha * sinAlpha);

        // compute ascending node
        final double cosISinAlpha = cosI * sinAlpha;
        final double u = cosAlpha * direction.getX() + cosISinAlpha * direction.getY();
        final double v = cosAlpha * direction.getY() - cosISinAlpha * direction.getX();

        return FastMath.atan2(v, u);

    }

    /** Fit inclination offsets.
     * @exception OrekitException if states cannot be converted to Earth frame
     */
    private void fitInclinationOffsets()
        throws OrekitException {

        for (final ErrorModel errorModel : inclinationModels) {
            errorModel.resetFitting(fitStart.getDate(), errorModel.getFittedParameters());
        }

        final CircularOrbit orbit = (CircularOrbit) OrbitType.CIRCULAR.convertType(fitStart.getOrbit());
        final double referenceN0 = orbit.getRightAscensionOfAscendingNode();
        final double referenceN1 = 2 * FastMath.PI / Constants.JULIAN_YEAR;

        for (final Crossing crossing : crossings) {

            if (!isEquatorialLatitude(crossing.getGridPoint().getGeodeticPoint().getLatitude())) {
                // non-equatorial latitude point, we can use its longitude as an indicator for offset

                // find current point and reference point in inertial frame
                final SpacecraftState state = crossing.getState();
                final Vector3D current      = state.getPVCoordinates().getPosition();
                final double referenceN     = referenceN0 + referenceN1 * state.getDate().durationFrom(orbit.getDate());

                // inclination for current point
                final double dt           = crossing.getDate().durationFrom(fitStart.getDate());
                final double deltaN       = fittedDN[0] + dt * (fittedDN[1] + dt * fittedDN[2]);
                final double currentI     = computeInclination(current.normalize(), referenceN + deltaN);

                // inclination for reference grid point
                final Transform transform = earth.getBodyFrame().getTransformTo(state.getFrame(), state.getDate());
                final Vector3D reference  = transform.transformPosition(crossing.getGridPoint().getCartesianPoint());
                final double referenceI   = computeInclination(reference.normalize(), referenceN);

                final double deltaI = currentI - referenceI;

                if (state.getDate().compareTo(freeInterval[0]) >= 0 &&
                    state.getDate().compareTo(freeInterval[1]) <= 0) {
                    for (final ErrorModel errorModel : inclinationModels) {
                        if (errorModel.matches(crossing.getGridPoint().getGeodeticPoint().getLatitude(),
                                               crossing.getGridPoint().isAscending())) {
                            errorModel.addPoint(crossing.getDate(), deltaI);
                        }
                    }
                }

            }

        }

        // fit all point/direction specific models and compute a mean model
        Arrays.fill(fittedDI, 0);
        int n = 0;
        for (final ErrorModel errorModel : inclinationModels) {
            if (!isEquatorialLatitude(errorModel.getLatitude())) {
                errorModel.fit();
                final double[] f = errorModel.approximateAsPolynomialOnly(1, fitStart.getDate(), 2, 2,
                                                                          fitStart.getDate(), freeInterval[1],
                                                                          meanPeriod);
                for (int i = 0; i < fittedDI.length; ++i) {
                    fittedDI[i] += f[i];
                }
                ++n;
            }
        }
        for (int i = 0; i < fittedDI.length; ++i) {
            fittedDI[i] /= n;
        }

    }

    /** Compute inclination.
     * <p>
     * The direction cosines or spacecraft in an inertial frames can be computed as:
     * <pre>
     *  x = cos(Omega) cos(alpha) - sin(Omega) cos(i) sin(alpha)
     *  y = sin(Omega) cos(alpha) + cos(Omega) cos(i) sin(alpha)
     *  z = sin(i) sin(alpha)
     * </pre>
     * so if we know these direction cosines (x, y, z) and ascending node Omega, we
     * can compute an intermediate rotated system:
     * <pre>
     *  u = cos(Omega) x + sin(Omega) y = cos(alpha)
     *  v = cos(Omega) y - sin(Omega) x = cos(i) sin(alpha)
     * </pre>
     * solving this system gives:
     * cos(alpha) = u, sin(alpha) = sign(z) * sqrt[1 - cos^2(alpha)]
     * once we know sin(alpha), we can compute:
     * <pre>
     * cos(i) = v / sin(alpha)
     * </pre>
     * hence we deduce i from (x, y, z) and an assumed value for Omega
     *</p>
     * @param direction normalized point direction in inertial frame
     * @param omega assumed right ascension of ascending node
     * @return inclination
     */
    private double computeInclination(final Vector3D direction, final double omega) {

        final double cosOmega = FastMath.cos(omega);
        final double sinOmega = FastMath.sin(omega);

        // compute latitude argument
        final double u = cosOmega * direction.getX() + sinOmega * direction.getY();
        final double v = cosOmega * direction.getY() - sinOmega * direction.getX();
        final double sinAlpha = FastMath.copySign(FastMath.sqrt(1 - u * u), direction.getZ());

        return FastMath.acos(v / sinAlpha);

    }

    /** Check if a latitude is an intermediate one or an extreme one.
     * @return true if latitude is an extreme latitude
     */
    private boolean isExtremeLatitude(final double latitude) {
        return latitude < -switchLatitude || latitude > switchLatitude;
    }

    /** Check if a latitude is close to equator.
     * @return true if latitude is close to equator
     */
    private boolean isEquatorialLatitude(final double latitude) {
        return latitude >= -SWITCH_LIMIT && latitude <= SWITCH_LIMIT;
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
    private static class ErrorModel extends SecularAndHarmonic {

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
