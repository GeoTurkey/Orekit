/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.CelestialBody;
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

    /** Safety margin on longitude error window. */
    private final double safetyMargin;

    /** Indicator for side targeting. */
    private boolean forceReset;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model out-of-plane maneuver model
     * @param maneuverSequence sequence of day-of-year and maneuver indices for maneuver switching
     * @param firstOffset time offset of the first maneuver with respect to cycle start
     * @param maxManeuvers maximum number of maneuvers to set up in one cycle
     * @param orbitsSeparation minimum time between split parts in number of orbits
     * @param earth Earth model
     * @param sun Sun model
     * @param referenceRadius reference radius of the Earth for the potential model (m)
     * @param mu central attraction coefficient (m<sup>3</sup>/s<sup>2</sup>)
     * @param j2 un-normalized zonal coefficient (about +1.08e-3 for Earth)
     * @param grid grid points
     * @param maxDistance maximal cross distance to ground track allowed
     * @param horizon time horizon duration
     * @param inclinationOffsetFineTuning strange correction necessary to have accurate GTE-to-inclination-error translation
     */
    public InPlaneGroundTrackGrid(final String name, final String controlledName, final int controlledIndex,
                           final TunableManeuver[] model, int[][] maneuverSequence, final double firstOffset,
                           final int maxManeuvers, final int orbitsSeparation,
                           final OneAxisEllipsoid earth, final CelestialBody sun,
                           final double referenceRadius, final double mu, final double j2,
                           final List<GridPoint> grid, final double maxDistance, final double horizon, double inclinationOffsetFineTuning)
        throws SkatException {
        super(name, controlledName, controlledIndex, model, maneuverSequence, firstOffset, maxManeuvers, orbitsSeparation,
              earth, sun, referenceRadius, mu, j2, grid, maxDistance, true, horizon, inclinationOffsetFineTuning);
        this.safetyMargin = 0.1 * maxDistance;
    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException, SkatException {

        baseInitializeRun(iteration, maneuvers, propagator, fixedManeuvers, start, end);
        if (iteration == 0) {
            forceReset = false;
        }

    }

    /** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException {

        if (fitStart == null) {
            // the free maneuvers interval was probably too short, no fitting could be performed
            // we cannot even know if current orbit is good or not, so we don't control anything
            return tunables;
        }

        final double dlMax    = getMax() / earth.getEquatorialRadius();
        final double dlSafety = (getMax() - safetyMargin) / earth.getEquatorialRadius();

        if (loopDetected()) {
            // we are stuck in a convergence loop, we cannot improve the current solution
            return tunables;
        }

        // which depends on current state
        final double newDLdot;
        if (forceReset || (fittedDN[2] < 0 && fittedDN[0] > dlMax) || (fittedDN[2] > 0 && fittedDN[0] < -dlMax)) {
            // the start point is already on the wrong side of the window

            // make sure once we select this option, we stick to it for all iterations
            forceReset = true;

            // the current cycle is already bad, we set up a target to start a new cycle
            // at time horizon with good initial conditions, and reach this target by changing dlDot(t0)
            final double targetT  = cycleEnd.durationFrom(fitStart.getDate()) + firstOffset;
            final double targetDL = FastMath.copySign(dlMax, fittedDN[2]);
            newDLdot = (targetDL - fittedDN[0]) / targetT - fittedDN[2] * targetT;

        } else {
            // the start point is on the right side of the window

            final double tPeak   = -0.5 * fittedDN[1] / fittedDN[2];
            final double dlPeak  = fittedDN[0] + 0.5 * fittedDN[1] * tPeak;
            final double finalT  = cycleEnd.durationFrom(fitStart.getDate()) + firstOffset;
            final double finalDL = fittedDN[0] + finalT * (fittedDN[1] + finalT * fittedDN[2]);

            final boolean intermediateExit = tPeak > 0 && tPeak < finalT && FastMath.abs(dlPeak) > dlMax;
            final boolean finalExit        = FastMath.abs(finalDL) >= dlMax;
            if (intermediateExit || finalExit) {
                // longitude error exits the window limit before next cycle

                // we target a future longitude error peak osculating window boundary
                // (taking a safety margin into account if possible)
                double targetDL = FastMath.copySign(FastMath.abs(fittedDN[0]) >= dlSafety ? dlMax : dlSafety,
                                                    -fittedDN[2]);
                newDLdot = FastMath.copySign(FastMath.sqrt(4 * fittedDN[2] * (fittedDN[0] - targetDL)),
                                             (tPeak > 0) ? fittedDN[1] : -fittedDN[1]);

            } else {
                // longitude error stays within bounds up to next cycle,
                // we don't change anything on the maneuvers
                return tunables;
            }

        }

        // compute semi major axis offset needed to achieve station-keeping target
        final double dlDotDa = -3 * FastMath.PI / (fitStart.getA() * Constants.JULIAN_DAY);
        final double deltaA  = (newDLdot - fittedDN[1]) / dlDotDa;

        final ScheduledManeuver[] tuned;
        final AdapterPropagator adapterPropagator = new AdapterPropagator(reference);
        if (iteration == 0) {
            // we need to first define the number of maneuvers and their initial settings

            // compute the in plane maneuver required to get this initial semi-major axis offset
            final double mu           = fitStart.getMu();
            final double a            = fitStart.getA();
            final double totalDeltaV  = thrustSignVelocity(fitStart,getModel()) *
                                        FastMath.sqrt(mu * (2 / a - 1 / (a + deltaA))) - FastMath.sqrt(mu / a);

            // in order to avoid tempering eccentricity too much,
            // we use a (n+1/2) orbits between maneuvers, where n is an integer
            final double separation = (orbitsSeparation + 0.5) * meanPeriod;

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
            final double deltaVChange = thrustSignVelocity(fitStart,getModel()) *
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
