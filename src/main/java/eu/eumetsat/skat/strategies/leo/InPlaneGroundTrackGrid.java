/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.Arrays;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;
import org.orekit.frames.Transform;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.AdapterPropagator;
import org.orekit.propagation.analytical.J2DifferentialEffect;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;

/**
 * Station-keeping control attempting to follow a specified ground-track grid.
 * <p>
 * The ground-track is defined by a set of latitude/longitude reference points
 * with an associated latitude crossing direction (ascending/descending). The
 * control checks the longitude offset with respect to the reference longitude
 * each time the spacecraft crosses a reference latitude in the specifed
 * direction. Hence the control correspond to an East/West positioning with respect
 * to the grid.
 * </p>
 * <p>
 * The longitude offset evolution is parabolic and is mainly due to atmospheric drag
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
public class InPlaneGroundTrackGrid extends AbstractGroundTrackGrid {

    /** Mean fitting parameters for longitude error. */
    private final double[] fittedDL;

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
    public InPlaneGroundTrackGrid(final String name, final String controlledName, final int controlledIndex,
                           final TunableManeuver model, final double firstOffset,
                           final int maxManeuvers, final int orbitsSeparation,
                           final OneAxisEllipsoid earth,
                           final double referenceRadius, final double mu, final double j2,
                           final List<GridPoint> grid, final double maxDistance, final double horizon)
        throws SkatException {
        super(name, controlledName, controlledIndex, model, firstOffset, maxManeuvers, orbitsSeparation,
              earth, referenceRadius, mu, j2, grid, maxDistance, horizon);

        this.fittedDL         = new double[3];
        this.safetyMargin     = 0.1 * maxDistance;

    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException, SkatException {

        if (!baseInitializeRun(iteration, maneuvers, propagator, fixedManeuvers, start, end)) {
            checkMargins(start, 0);
        }
        if (iteration == 0) {
            forceReset = false;
        }

        // fit longitude offsets to parabolic models
        for (final ErrorModel errorModel : errorModels) {
            errorModel.resetFitting(fitStart.getDate(), errorModel.getFittedParameters());
        }
        for (final Crossing crossing : crossings) {
            if (isIntermediateLatitude(crossing.getGridPoint().getGeodeticPoint().getLatitude())) {
                // intermediate latitude point, we can use its longitude as an indicator for offset
                final SpacecraftState state = crossing.getState();
                final Transform transform   = state.getFrame().getTransformTo(earth.getBodyFrame(), state.getDate());
                final Vector3D current      = transform.transformPosition(state.getPVCoordinates().getPosition());
                final GeodeticPoint gp      = earth.transform(current, earth.getBodyFrame(), state.getDate());

                final double l1     = gp.getLongitude();
                final double l2     = crossing.getGridPoint().getGeodeticPoint().getLongitude();
                final double deltaL = MathUtils.normalizeAngle(l1 - l2, 0.0);

                // check the distances
                final Vector3D subSat = earth.transform(new GeodeticPoint(gp.getLatitude(), gp.getLongitude(), 0.0));
                final double distance = Vector3D.distance(subSat, crossing.getGridPoint().getCartesianPoint());
                checkMargins(state.getDate(), FastMath.copySign(distance, deltaL));

                for (final ErrorModel errorModel : errorModels) {
                    if (errorModel.matches(crossing.getGridPoint().getGeodeticPoint().getLatitude(),
                                           crossing.getGridPoint().isAscending())) {
                        errorModel.addPoint(crossing.getDate(), deltaL);
                    }
                }

            }
        }

        // fit all point/direction specific models and compute a mean model
        Arrays.fill(fittedDL, 0);
        for (final ErrorModel errorModel : errorModels) {
            errorModel.fit();
            final double[] f = errorModel.approximateAsPolynomialOnly(2, fitStart.getDate(), 2, 2,
                                                                      fitStart.getDate(),
                                                                      freeInterval[1],
                                                                      fitStart.getKeplerianPeriod());
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

}
