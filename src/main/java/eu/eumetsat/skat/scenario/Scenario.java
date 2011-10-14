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
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.MonitorableMonoSKData;
import eu.eumetsat.skat.utils.SkatException;

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
    private final double duration;

    /** Output step for monitoring. */
    private final double outputstep;

    /** EME 2000 Frame. */
    private final Frame eme2000;

    /** Earth model. */
    private final BodyShape earth;

    /** Sun model. */
    private final CelestialBody sun;

    /** Monitored values. */
    private final List<MonitorableMonoSKData> monitorables;

    /** Simple constructor.
     * <p>
     * Create an empty scenario without any components. Components
     * must be added by calling {@link #addComponent(ScenarioComponent)}.
     * </p>
     * @param cycleDuration duration of one cycle (s)
     * @param outputStep output step for monitoring (s)
     * @param propagator propagator to use
     * @param earth Earth model
     */
    public Scenario(final double cycleDuration, final double outputStep,
                    final BodyShape earth, final CelestialBody sun) {
        this.components   = new ArrayList<ScenarioComponent>();
        this.duration     = cycleDuration;
        this.outputstep   = outputStep;
        this.eme2000      = FramesFactory.getEME2000();
        this.earth        = earth;
        this.sun          = sun;
        this.monitorables = new ArrayList<MonitorableMonoSKData>();
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

    /** Monitor a station-keeping data.
     * @param key key of station-keeping data to monitor
     */
    public void monitor(final MonitorableMonoSKData key) {
        monitorables.add(key);
    }

    /** {@inheritDoc}
     * <p>The scenario will be run in loops until the target date
     * is reached. At each iteration of the loop, the target date of
     * the iteration will be set according to the cycle duration set at
     * construction.</p>
     */
    public ScenarioState[] updateStates(final ScenarioState[] originals, final AbsoluteDate target)
        throws OrekitException {

        ScenarioState[] states = originals.clone();
        AbsoluteDate iterationTarget;
        do {

            // set target date for iteration using cycle duration
            iterationTarget = states[0].getEstimatedStartState().getDate().shiftedBy(duration);

            // run all components of the scenario in order
            for (final ScenarioComponent component : components) {
                states = component.updateStates(states, iterationTarget);
            }

            // monitor data
            AbsoluteDate tMin = originals[0].getEphemeris().getMinDate();
            AbsoluteDate tMax = originals[0].getEphemeris().getMaxDate();
            for (AbsoluteDate date = tMin; date.compareTo(tMax) <= 0; date = date.shiftedBy(outputstep)) {

                // check for maneuvers that have occurred
                updatePendingManeuvers(date, states);

                // check for nodes
                updateNodes(date, states);

                // perform monitoring
                for (final MonitorableMonoSKData monitorable : monitorables) {
                    monitorable.update(states, earth);
                }

            }

            // prepare next cycle
            for (int i = 0; i < states.length; ++i) {
                states[i] = states[i].updateCyclesNumber(states[i].getCyclesNumber() + 1);
                states[i] = states[i].updateRealStartState(states[i].getRealEndState());
                states[i] = states[i].updateEstimatedStartState(null);
                states[i] = states[i].updateRealEndState(null);
                states[i] = states[i].updateTheoreticalManeuvers(new ArrayList<ScheduledManeuver>());
                states[i] = states[i].updatePerformedManeuvers(new ArrayList<ScheduledManeuver>());
            }

        } while (iterationTarget.compareTo(target) < 0);

        return states;

    }

    /** Update the maneuvers state up to current date.
     * @param date current date
     * @param states states array to update
     * @exception OrekitException if a maneuver cannot be checked
     */
    private void updatePendingManeuvers(final AbsoluteDate date, final ScenarioState[] states)
        throws OrekitException {

        final AbsoluteDate previous = date.shiftedBy(-outputstep);

        for (int i = 0; i < states.length; ++i) {
            for (final ScheduledManeuver maneuver : states[i].getPerformedManeuvers()) {
                if ((maneuver.getDate().compareTo(previous) > 0) &&
                    (maneuver.getDate().compareTo(date) <= 0)) {
                    // the maneuver occurred during last step, take it into account
                    if (maneuver.isInPlane()) {
                        states[i] = states[i].updateInPlaneManeuvers(states[i].getInPlane() + 1,
                                                                     states[i].getInPlaneDV() +
                                                                     maneuver.getDeltaV().getNorm());
                    } else {
                        states[i] = states[i].updateOutOfPlaneManeuvers(states[i].getOutOfPlane() + 1,
                                                                        states[i].getOutOfPlaneDV() +
                                                                        maneuver.getDeltaV().getNorm());
                    }
                }
            }
        }

    }

    /** Update the nodes up to current date.
     * @param date current date
     * @param states states array to update
     * @exception OrekitException if a node cannot be computed
     */
    private void updateNodes(final AbsoluteDate date, final ScenarioState[] states)
        throws OrekitException {
        for (int i = 0; i < states.length; ++i) {
            final double dtA = date.durationFrom(states[i].getAscendingNodeDate());
            final double dtD = date.durationFrom(states[i].getDescendingNodeDate());
            if (dtA * dtD > 0) {

                // the stored nodes do not bracket current date anymore, recompute them
                final SpacecraftState firstState = findClosestNode(states[i].getEphemeris(), date);
                final AbsoluteDate firstDate = firstState.getDate();
                final double period = firstState.getKeplerianPeriod();
                final double dt = (firstDate.compareTo(date) <= 0) ? 0.5 * period : -0.5 * period;
                final SpacecraftState secondState = findClosestNode(states[i].getEphemeris(), firstDate.shiftedBy(dt));
                final AbsoluteDate secondDate = secondState.getDate();

                if (date.durationFrom(firstDate) * date.durationFrom(secondDate) > 0) {
                    // we failed to bracket date between two nodes, this should never happen
                    throw SkatException.createInternalError(null);
                }

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
            final double alphaM = MathUtils.normalizeAngle(orbit.getAlphaM(), 0);
            final double n      = orbit.getKeplerianMeanMotion();
            final double dt     = ((FastMath.abs(alphaM) <= FastMath.PI) ? -alphaM : FastMath.PI - alphaM) / n;

            // set up search range to the half orbit around node
            final double ephemerideMin  = ephemeris.getMinDate().durationFrom(date);
            final double ephemerideMax  = ephemeris.getMaxDate().durationFrom(date);
            final double halfSearchSpan = FastMath.PI / (2 * n);
            double dtMin = FastMath.max(ephemerideMin, dt - halfSearchSpan);
            double dtMax = FastMath.min(ephemerideMax, dtMin + 2 * halfSearchSpan);
            dtMin = dtMax - 2 * halfSearchSpan;

            // search the node, defined by spacecraft crossing equator in EME2000
            final UnivariateRealSolver solver = new BracketingNthOrderBrentSolver(1.0e-6, 5);
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
