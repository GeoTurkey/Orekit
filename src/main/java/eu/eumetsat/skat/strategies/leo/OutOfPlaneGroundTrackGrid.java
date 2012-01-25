/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.OrbitType;
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
 * control checks the out of plane offset with respect to the reference point
 * each time the spacecraft crosses a reference latitude in the specifed
 * direction.
 * </p>
 * @author Luc Maisonobe
 */
public class OutOfPlaneGroundTrackGrid extends AbstractGroundTrackGrid {

    /** Reference state at first node in eclipse. */
    private SpacecraftState nodeState;

    /** Sun model. */
    private CelestialBody sun;

    /** Indicator for compensating long burns inefficiency. */
    private boolean compensateLongBurn;

    /** Safety margin. */
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
     * @param sun Sun model
     * @param referenceRadius reference radius of the Earth for the potential model (m)
     * @param mu central attraction coefficient (m<sup>3</sup>/s<sup>2</sup>)
     * @param j2 un-normalized zonal coefficient (about +1.08e-3 for Earth)
     * @param grid grid points
     * @param maxDistance maximal cross distance to ground track allowed
     * @param horizon time horizon duration
     * @param compensateLongBurn if true, long burn inefficiency should be compensated
     */
    public OutOfPlaneGroundTrackGrid(final String name, final String controlledName, final int controlledIndex,
                                     final TunableManeuver model, final double firstOffset,
                                     final int maxManeuvers, final int orbitsSeparation,
                                     final OneAxisEllipsoid earth, final CelestialBody sun,
                                     final double referenceRadius, final double mu, final double j2,
                                     final List<GridPoint> grid, final double maxDistance, final double horizon,
                                     final boolean compensateLongBurn)
        throws SkatException {
        super(name, controlledName, controlledIndex, model, firstOffset, maxManeuvers, orbitsSeparation,
              earth, referenceRadius, mu, j2, grid, maxDistance, horizon);
        this.sun                = sun;
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

            forceReset = false;

            // look for a node at which maneuvers can be performed
            nodeState = null;
            for (int i = 0; nodeState == null && i < crossings.size(); ++i) {

                // find the first available node that is in eclipse for maneuver
                final double stepSize = meanPeriod / 100;
                nodeState = firstLatitudeCrossing(0.0, true, earth, freeInterval[0], freeInterval[1],
                                                  stepSize, propagator);
                Vector3D satPos = nodeState.getPVCoordinates().getPosition();
                Vector3D sunPos = sun.getPVCoordinates(nodeState.getDate(), nodeState.getFrame()).getPosition();
                if (Vector3D.dotProduct(satPos, sunPos) > 0) {
                    // wrong node, it is under Sun light, select the next one
                    nodeState = latitudeCrossing(0.0, earth, nodeState.getDate().shiftedBy(0.5 * meanPeriod),
                                                 freeInterval[1], stepSize, meanPeriod / 8, propagator);
                }

                // ensure maneuvers separation requirements
                while (nodeState.getDate().durationFrom(freeInterval[0]) < firstOffset) {
                    nodeState = latitudeCrossing(0.0, earth, nodeState.getDate().shiftedBy(meanPeriod),
                                                 freeInterval[1], stepSize, meanPeriod / 8, propagator);
                }

            }

        }

        fitOffsets();

        if (getMargins() > 0) {
            // there are some margins, don't perform any maneuvers
            nodeState = null;
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

        double newIdot;
        if (forceReset || (fittedDI[2] < 0 && fittedDI[0] > diMax) || (fittedDI[2] > 0 && fittedDI[0] < -diMax)) {
            // the start point is already on the wrong side of the window

            // make sure once we select this option, we stick to it for all iterations
            forceReset = true;

            // the current cycle is already bad, we set up a target to start a new cycle
            // at time horizon with good initial conditions, and reach this target by changing iDot(t0)
            final double targetT = cycleEnd.durationFrom(nodeState.getDate()) + firstOffset;
            final double targetI = (fittedDI[2] < 0) ? (-diMax + safetyMargin) : (diMax - safetyMargin);
            newIdot = (targetI - fittedDI[0]) / targetT - fittedDI[2] * targetT;

        } else {
            // the start point is on the right side of the window
            final double tPeak   = -0.5 * fittedDI[1] / fittedDI[2];
            final double iPeak   = fittedDI[0] + 0.5 * fittedDI[1] * tPeak;
            final double finalT = cycleEnd.durationFrom(nodeState.getDate()) + firstOffset;
            final double finalI = fittedDI[0] + finalT * (fittedDI[1] + finalT * fittedDI[2]);

            final boolean intermediateExit = tPeak > 0 && tPeak < finalT && (iPeak > diMax || iPeak < -diMax);
            final boolean finalExit        = finalI >= diMax || finalI <= -diMax;
            if (intermediateExit || finalExit) {
                // longitude offset exits the window limits

                // we target a future longitude offset peak osculating window boundary
                // (taking a safety margin into account if possible)
                double targetI;
                if (fittedDI[2] <= 0) {
                    targetI = diMax - safetyMargin;
                    if (fittedDI[0] >= targetI) {
                        targetI = diMax;
                    }
                } else {
                    targetI = -diMax + safetyMargin;
                    if (fittedDI[0] <= targetI) {
                        targetI = -diMax;
                    }
                }
                newIdot = FastMath.copySign(FastMath.sqrt(4 * fittedDI[2] * (fittedDI[0] - targetI)),
                                            (tPeak > 0) ? fittedDI[1] : -fittedDI[1]);

            } else {
                // longitude offset stays within bounds up to next cycle,
                // we don't change anything on the maneuvers
                nodeState = null;
                return tunables;
            }

        }

        // linearized relationship between hDot and inclination in the neighborhood of maneuver
        // the solar time h(t) is approximately h(tn) = Omega(tn) + eta - (pi + alpha(tn))
        // where Omega is spacecraft right ascension of ascending node, eta is right ascension
        // difference between spacecraft at specified latitude and ascending node, and alpha is
        // Sun right ascension. Eta and pi are fixed, so we get the time derivative
        // hDot = raanDot - alphaDot. Raan time derivative due to J2 is proportional to cos(i),
        // so hDot = k cos(i) - alphaDot (with both k and cos(i) negative for sun synchronous
        // orbits so hDot is close to zero) and hence dhDot / di = -k sin (i) = -(alphaDot + hDot) tan(i)
        final double dhRadDotDi = -(2 * FastMath.PI / Constants.JULIAN_YEAR + fittedDI[1]) * FastMath.tan(nodeState.getI());
        final double dhDotDi    = 12 * dhRadDotDi / FastMath.PI;

        // compute inclination offset needed to achieve station-keeping target
        final double deltaI = (newIdot - fittedDI[1]) / dhDotDi;

        final ScheduledManeuver[] tuned;
        final AdapterPropagator adapterPropagator = new AdapterPropagator(reference);
        if (iteration == 0) {
            // we need to first define the number of maneuvers and their initial settings

            // compute the out of plane maneuver required to get the initial inclination offset
            final Vector3D v          = nodeState.getPVCoordinates().getVelocity();
            final double totalDeltaV  = thrustSignMomentum(nodeState) * FastMath.signum(v.getZ()) *
                                        v.getNorm() * deltaI;

            // compute the number of maneuvers required
            final double separation = orbitsSeparation * meanPeriod;
            final TunableManeuver model = getModel();
            final double limitDV = (totalDeltaV < 0) ? model.getDVInf() : model.getDVSup();
            int nMan = FastMath.min(maxManeuvers, (int) FastMath.ceil(FastMath.abs(totalDeltaV / limitDV)));
            while (nodeState.getDate().shiftedBy((nMan - 1) * separation).getDate().compareTo(cycleEnd) >= 0) {
                --nMan;
            }
            final double deltaV = FastMath.max(model.getDVInf(), FastMath.min(model.getDVSup(), totalDeltaV / nMan));

            tuned = new ScheduledManeuver[tunables.length + nMan];
            System.arraycopy(tunables, 0, tuned, 0, tunables.length);
            changeTrajectory(tuned, 0, tunables.length, adapterPropagator);


            // add the new maneuvers
            for (int i = 0; i < nMan; ++i) {
                tuned[tunables.length + i] =
                        new ScheduledManeuver(model, nodeState.getDate().shiftedBy(i * separation),
                                              new Vector3D(deltaV, model.getDirection()),
                                              model.getCurrentThrust(), model.getCurrentISP(),
                                              adapterPropagator, false);
            }

        } else {

            // adjust the existing maneuvers

            // compute the out of plane maneuver required to get the initial inclination offset
            final double v            = nodeState.getPVCoordinates().getVelocity().getNorm();
            final double deltaVChange = thrustSignMomentum(nodeState) * v * deltaI;

            // distribute the change over all maneuvers
            tuned = tunables.clone();
            changeTrajectory(tuned, 0, tuned.length, adapterPropagator);
            distributeDV(deltaVChange, 0.0, tuned, adapterPropagator);

        }

        if (compensateLongBurn) {
            for (int i = 0; i < tuned.length; ++i) {
                if (tuned[i].getName().equals(getModel().getName())) {
                    tuned[i] = longBurnCompensation(tuned[i]);
                }
            }
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

    /** Compensate inefficiency of long burns.
     * @param maneuver maneuver to compensate
     * @return compensated maneuver (Isp reduced to get same dV with more consumed mass)
     * @throws PropagationException if state cannot be propagated around maneuvera
     */
    private ScheduledManeuver longBurnCompensation(final ScheduledManeuver maneuver)
        throws PropagationException {
        // this is a long out of plane maneuver, we adapt Isp to reflect
        // the fact more mass will be consumed to achieve the same velocity increment
        final double nominalDuration = maneuver.getDuration(maneuver.getStateBefore().getMass());

        final SpacecraftState startState = maneuver.getState(-0.5 * nominalDuration);
        final CircularOrbit startOrbit   = (CircularOrbit) (OrbitType.CIRCULAR.convertType(startState.getOrbit()));
        final double alphaS              = startOrbit.getAlphaV();

        final SpacecraftState endState   = maneuver.getState(+0.5 * nominalDuration);
        final CircularOrbit endOrbit     = (CircularOrbit) (OrbitType.CIRCULAR.convertType(endState.getOrbit()));
        final double alphaE              = endOrbit.getAlphaV();

        final double reductionFactor = (FastMath.sin(alphaE) - FastMath.sin(alphaS)) / (alphaE - alphaS);
        return new ScheduledManeuver(maneuver.getModel(), maneuver.getDate(),
                                     maneuver.getDeltaV(), maneuver.getThrust(),
                                     reductionFactor * getModel().getCurrentISP(),
                                     maneuver.getTrajectory(), maneuver.isReplanned());
    }

}
