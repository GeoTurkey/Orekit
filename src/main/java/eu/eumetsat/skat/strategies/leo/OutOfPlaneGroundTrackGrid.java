/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.List;

import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;

/**
 * Station-keeping control attempting to follow a specified ground-track grid.
 * <p>
 * The ground-track is defined by a set of latitude/longitude reference points
 * with an associated latitude crossing direction (ascending/descending). The
 * control checks the out of plane offset with respect to the reference point
 * each time the spacecraft crosses a reference latitude in the specifed
 * direction.
 * </p>
 * @author Luc Maisonobe
 */
public class OutOfPlaneGroundTrackGrid extends AbstractGroundTrackGrid {

    /** Reference state at first node in eclipse. */
    private SpacecraftState nodeState;

    /** Indicator for compensating long burns inefficiency. */
    private boolean compensateLongBurn;

    /** Safety margin. */
    private final double safetyMargin;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model out-of-plane maneuver model
     * @param maneuverSequence 
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
     * @param compensateLongBurn if true, long burn inefficiency should be compensated
     * @param inclinationOffsetFineTuning strange correction necessary to have accurate GTE-to-inclination-error translation
     */
    public OutOfPlaneGroundTrackGrid(final String name, final String controlledName, final int controlledIndex,
                                     final TunableManeuver[] model, int[][] maneuverSequence, final double firstOffset,
                                     final int maxManeuvers, final int orbitsSeparation,
                                     final OneAxisEllipsoid earth, final CelestialBody sun,
                                     final double referenceRadius, final double mu, final double j2,
                                     final List<GridPoint> grid, final double maxDistance, final double horizon,
                                     final boolean compensateLongBurn, final double inclinationOffsetFineTuning)
        throws SkatException {
        super(name, controlledName, controlledIndex, model, maneuverSequence, firstOffset, maxManeuvers, orbitsSeparation,
              earth, sun, referenceRadius, mu, j2, grid, maxDistance, false, horizon, inclinationOffsetFineTuning);
        this.safetyMargin       = 0.1 * maxDistance / earth.getEquatorialRadius();
        this.compensateLongBurn = compensateLongBurn;

    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException, SkatException {

        baseInitializeRun(iteration, maneuvers, propagator, fixedManeuvers, start, end);

        if (iteration == 0) {

            nodeState = null;

            if (fitStart != null) {
                final double diMax = getMax() / earth.getEquatorialRadius();
                if (FastMath.abs(fittedDI[0] + fittedDI[1] * end.durationFrom(fitStart.getDate())) > diMax) {
                    // inclination error exceeds boundaries
                    // look for a node at which maneuvers can be performed
                    nodeState = findManeuverNode(start, end, propagator);
                }
            }

        }

    }

    /** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException {

        final double diMax = getMax() / earth.getEquatorialRadius();
        if (nodeState == null) {
            // no maneuvers needed
            return tunables;
        }

        if (loopDetected()) {
            // we are stuck in a convergence loop, we cannot improve the current solution
            return tunables;
        }

        // we want to start inclination drift from the boundary
        final double deltaI = FastMath.copySign(diMax - safetyMargin, -fittedDI[1]) -
                              (fittedDI[0] + fittedDI[1] * nodeState.getDate().durationFrom(fitStart.getDate()));

        return tuneInclinationManeuver(tunables, reference, nodeState, deltaI, compensateLongBurn);

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
