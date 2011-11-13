/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.scenario;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.analysis.UnivariateRealFunction;
import org.apache.commons.math.analysis.solvers.BracketingNthOrderBrentSolver;
import org.apache.commons.math.analysis.solvers.UnivariateRealSolver;
import org.apache.commons.math.exception.NoBracketingException;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBody;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.MonitorableDuoSKData;
import eu.eumetsat.skat.utils.MonitorableMonoSKData;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/** Station-Keeping scenario.
 * <p>
 * A station-keeping scenario is a simple list of cycle components
 * that are run one after the other at each cycle, and then the cycle
 * is repeated.
 * </p>
 */
public class Scenario implements ScenarioComponent {

    /** Scenario components. */
    private final List<ScenarioComponent> components;

    /** Cycle duration. */
    private final double cycleDuration;

    /** Output step for monitoring. */
    private final double outputstep;

    /** EME 2000 Frame. */
    private final Frame eme2000;

    /** Earth model. */
    private final BodyShape earth;

    /** Sun model. */
    private final CelestialBody sun;

    /** Cycle end date. */
    private AbsoluteDate cycleEnd;

    /** Reference ground location. */
    private TopocentricFrame groundLocation;

    /** Mono-spacecraft monitorables. */
    private List<MonitorableMonoSKData> monitorablesMono;

    /** Duo-spacecrafts monitorables. */
    private List<MonitorableDuoSKData> monitorablesDuo;

    /** Simple constructor.
     * <p>
     * Create an empty scenario without any components. Components
     * must be added by calling {@link #addComponent(ScenarioComponent)}.
     * </p>
     * @param cycleDuration duration of one cycle (s)
     * @param outputStep output step for monitoring (s)
     * @param earth Earth model
     * @param sun Sun model
     * @param groundLocation reference ground location
     * @param monitorablesMono list of monitorables for mono-spacecraft
     * @param monitorablesDuo list of monitorables for duo-spacecrafts
     */
    public Scenario(final double cycleDuration, final double outputStep,
                    final BodyShape earth, final CelestialBody sun,
                    final TopocentricFrame groundLocation,
                    final List<MonitorableMonoSKData> monitorablesMono,
                    final List<MonitorableDuoSKData> monitorablesDuo) {
        this.components       = new ArrayList<ScenarioComponent>();
        this.cycleDuration    = cycleDuration;
        this.outputstep       = outputStep;
        this.eme2000          = FramesFactory.getEME2000();
        this.earth            = earth;
        this.sun              = sun;
        this.groundLocation   = groundLocation;
        this.monitorablesMono = monitorablesMono;
        this.monitorablesDuo  = monitorablesDuo;
    }

    /** Add a cycle component.
     * <p>
     * Cycle components must be added in simulation order.
     * </p>
     * @param component cycle component
     */
    public void addComponent(final ScenarioComponent component) {
        components.add(component);
    }

    /** {@inheritDoc} */
    public void setCycleEnd(final AbsoluteDate cycleEnd) {
        this.cycleEnd = cycleEnd;
    }

    /** {@inheritDoc}
     * <p>The scenario will be run in loops until the target date
     * is reached. At each iteration of the loop, the target date of
     * the iteration will be set according to the cycle duration set at
     * construction.</p>
     */
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException, SkatException {

        ScenarioState[] states = originals.clone();
        AbsoluteDate iterationTarget;
        do {

            // set target date for iteration using cycle duration
            iterationTarget = states[0].getRealStartState().getDate().shiftedBy(cycleDuration);

            // run all components of the scenario in order
            for (final ScenarioComponent component : components) {
                component.setCycleEnd(iterationTarget);
                states = component.updateStates(states);
            }

            // monitor data
            AbsoluteDate tMin = states[0].getPerformedEphemeris().getMinDate();
            AbsoluteDate tMax = states[0].getPerformedEphemeris().getMaxDate();
            for (AbsoluteDate date = tMin; date.compareTo(tMax) <= 0; date = date.shiftedBy(outputstep)) {

                // check for maneuvers that have occurred
                updatePendingManeuvers(date, states);

                // check for nodes
                updateNodes(date, states);

                // perform monitoring
                for (final MonitorableMonoSKData monitorable : monitorablesMono) {
                    monitorable.update(date, states, earth);
                }
                for (final MonitorableDuoSKData monitorable : monitorablesDuo) {
                    monitorable.update(date, states, earth, groundLocation);
                }

            }

            // prepare next cycle
            for (int i = 0; i < states.length; ++i) {
                if (states[i].getRealEndState() == null) {
                    throw new SkatException(SkatMessages.NO_END_STATE,
                                            states[i].getName(), states[i].getCyclesNumber());
                }
                states[i] = states[i].updateCyclesNumber(states[i].getCyclesNumber() + 1);
                states[i] = states[i].updateInPlaneManeuvers(states[i].getInPlaneManeuvers(),
                                                             0.0,
                                                             states[i].getInPlaneTotalDV());
                states[i] = states[i].updateOutOfPlaneManeuvers(states[i].getOutOfPlaneManeuvers(),
                                                                0.0,
                                                                states[i].getOutOfPlaneTotalDV());
                states[i] = states[i].updateRealStartState(states[i].getRealEndState());
                states[i] = states[i].updateEstimatedStartState(null);
                states[i] = states[i].updateRealEndState(null);
                states[i] = states[i].updateManeuvers(null);
            }

        } while (iterationTarget.compareTo(cycleEnd) < 0);

        return states;

    }

    /** Update the maneuvers state up to current date.
     * @param date current date
     * @param states states array to update
     * @exception OrekitException if a maneuver cannot be checked
     * @exception SkatException if there are no maneuvers
     */
    private void updatePendingManeuvers(final AbsoluteDate date, final ScenarioState[] states)
        throws OrekitException, SkatException {

        final AbsoluteDate previous = date.shiftedBy(-outputstep);

        for (int i = 0; i < states.length; ++i) {
            if (states[i].getManeuvers() == null) {
                throw new SkatException(SkatMessages.NO_MANEUVERS_STATE,
                                        states[i].getName(), states[i].getCyclesNumber());
            }
            for (final ScheduledManeuver maneuver : states[i].getManeuvers()) {
                if ((maneuver.getDate().compareTo(previous) > 0) &&
                    (maneuver.getDate().compareTo(date) <= 0)) {
                    // the maneuver occurred during last step, take it into account
                    final double dv = maneuver.getDeltaV().getNorm();
                    if (maneuver.isInPlane()) {
                        states[i] = states[i].updateInPlaneManeuvers(states[i].getInPlaneManeuvers() + 1,
                                                                     states[i].getInPlaneCycleDV() + dv,
                                                                     states[i].getInPlaneTotalDV() + dv);
                    } else {
                        states[i] = states[i].updateOutOfPlaneManeuvers(states[i].getOutOfPlaneManeuvers() + 1,
                                                                        states[i].getOutOfPlaneCycleDV() + dv,
                                                                        states[i].getOutOfPlaneTotalDV() + dv);
                    }
                }
            }
        }

    }

    /** Update the nodes up to current date.
     * @param date current date
     * @param states states array to update
     * @exception OrekitException if a node cannot be computed
     * @exception SkatException if there are no ephemerides
     */
    private void updateNodes(final AbsoluteDate date, final ScenarioState[] states)
        throws OrekitException, SkatException {
        for (int i = 0; i < states.length; ++i) {
            final BoundedPropagator ephemeris = states[i].getPerformedEphemeris();
            if (ephemeris == null) {
                throw new SkatException(SkatMessages.NO_EPHEMERIS_STATE,
                                        states[i].getName(), states[i].getCyclesNumber());
            }
            final double dtA = date.durationFrom(states[i].getAscendingNodeDate());
            final double dtD = date.durationFrom(states[i].getDescendingNodeDate());
            final double margin = 0.001; // margin set up to avoid searching out of ephemeris range
            if (dtA * dtD > 0 &&
                date.durationFrom(ephemeris.getMinDate()) >  margin &&
                date.durationFrom(ephemeris.getMaxDate()) < -margin) {

                // the stored nodes do not bracket current date anymore, recompute them
                final SpacecraftState firstState = findClosestNode(ephemeris, date);
                final AbsoluteDate firstDate = firstState.getDate();
                final double halfPeriod = 0.5 * firstState.getKeplerianPeriod();
                final double dt;
                if (firstDate.compareTo(date) <= 0) {
                    // first node is before the date, the bracketing other node is therefore after it
                    // are we allowed to select this one and ensure bracketing ?
                    if (ephemeris.getMaxDate().durationFrom(firstDate) > halfPeriod) {
                        // OK, we can bracket the date
                        dt =  halfPeriod;
                    } else {
                        // no, the ephemeris is not long enough, we must select an earlier node
                        dt = -halfPeriod;
                    }
                } else {
                    // first node is after the date, the bracketing other node is therefore before it
                    // are we allowed to select this one and ensure bracketing ?
                    if (firstDate.durationFrom(ephemeris.getMinDate()) > halfPeriod) {
                        // OK, we can bracket the date
                        dt = -halfPeriod;
                    } else {
                        // no, the ephemeris is not long enough, we must select a later node
                        dt =  halfPeriod;
                    }
                }
                final SpacecraftState secondState = findClosestNode(ephemeris, firstDate.shiftedBy(dt));
                final AbsoluteDate secondDate = secondState.getDate();

                final double firstSolarTime  = solarTime(firstState);
                final double secondSolarTime = solarTime(secondState);

                // update the state
                if (firstState.getPVCoordinates(eme2000).getVelocity().getZ() >= 0) {
                    // first state is an ascending node
                    states[i] = states[i].updateAscendingNodeCrossing(firstDate,   firstSolarTime);
                    states[i] = states[i].updateDescendingNodeCrossing(secondDate, secondSolarTime);
                } else {
                    // first state is a descending node
                    states[i] = states[i].updateDescendingNodeCrossing(firstDate,  firstSolarTime);
                    states[i] = states[i].updateAscendingNodeCrossing(secondDate,  secondSolarTime);
                }

            }
        }
    }

    /** Find the closest node.
     * @param ephemeris spacecraft ephemeris
     * @param date current date
     * @return spacecraft state at node
     * @exception OrekitException if ephemeride cannot be propagated at some date
     */
    private SpacecraftState findClosestNode(final BoundedPropagator ephemeris,
                                            final AbsoluteDate date)
        throws OrekitException {

        try {
            // get the orbit at curent date
            final CircularOrbit orbit =
                    (CircularOrbit) OrbitType.CIRCULAR.convertType(ephemeris.propagate(date).getOrbit());

            // rough estimate of time to closest node using Keplerian motion
            final double halfPi        = 0.5 * FastMath.PI;
            final double alphaM        = MathUtils.normalizeAngle(orbit.getAlphaM(), 0);
            final double n             = orbit.getKeplerianMeanMotion();
            final double targetAlphaM  =
                (alphaM < -halfPi) ? -FastMath.PI : ((alphaM < halfPi) ? 0 : FastMath.PI);
            double dt                  = (targetAlphaM - alphaM) / n;
            final double quarterPeriod = halfPi / n;

            // set up search range to the half orbit around node, within ephemeris
            final double margin        = 1.0e-3;
            final double ephemerisMin  = ephemeris.getMinDate().durationFrom(date);
            final double ephemerisMax  = ephemeris.getMaxDate().durationFrom(date);
            double dtMin;
            double dtMax;
            if (dt <= ephemerisMin + quarterPeriod) {
                // the closest node lies near ephemeris start
                dtMin = ephemerisMin + margin;
                dt    = dtMin + quarterPeriod;
                dtMax = dt + quarterPeriod;
            } else if (dt >= ephemerisMax - quarterPeriod) {
                // the closest node lies after ephemeris, just use one half-period at ephemeris end
                dtMax = ephemerisMax - margin;
                dt    = dtMax - quarterPeriod;
                dtMin = dt - quarterPeriod;
            } else {
                // the closest node is well within ephemeris range
                dtMin = dt - quarterPeriod;
                dtMax = dt + quarterPeriod;
            }

            // search the node, defined by spacecraft crossing equator in EME2000
            final UnivariateRealSolver solver = new BracketingNthOrderBrentSolver(1.0e-3, 5);
            double dtNode = solver.solve(1000, new UnivariateRealFunction() {
                /** {@inheritDoc} */
                public double value(double x) {
                    try {
                        // estimate spacecraft position with respect to equator
                        final SpacecraftState state = ephemeris.propagate(date.shiftedBy(x));
                        return state.getPVCoordinates(eme2000).getPosition().getZ();
                    } catch (OrekitException oe) {
                        throw new OrekitExceptionWrapper(oe);
                    }
                }
            }, dtMin, dtMax, dt);

            return ephemeris.propagate(date.shiftedBy(dtNode));
        } catch (OrekitExceptionWrapper oew) {
            throw oew.getException();
        } catch (NoBracketingException nbe) {
            throw new OrekitException(nbe);
        }
        
    }

    /** Compute solar time.
     * @param state spacecraft state
     * @return spacecraft local solar time (between 0 and 24 hours)
     * @throws OrekitException if sun or spacecraft positions cannot be determined
     */
    private double solarTime(final SpacecraftState state)
        throws OrekitException {
        final Vector3D spacecraftPosition = state.getPVCoordinates(eme2000).getPosition();
        final Vector3D sunPosition = sun.getPVCoordinates(state.getDate(), eme2000).getPosition();
        final double dAlpha = FastMath.PI + spacecraftPosition.getAlpha() - sunPosition.getAlpha();
        return 12.0 * MathUtils.normalizeAngle(dAlpha, FastMath.PI) / FastMath.PI;
    }

}
