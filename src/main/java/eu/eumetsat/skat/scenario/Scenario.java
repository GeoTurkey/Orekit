/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.scenario;

import java.util.ArrayList;
import java.util.List;

import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.time.AbsoluteDate;

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

    /** Simple constructor.
     * <p>
     * Create an empty scenario without any components. Components
     * must be added by calling {@link #addComponent(ScenarioComponent)}.
     * </p>
     * @param cycleDuration duration of one cycle (s)
     */
    public Scenario(final double cycleDuration) {
        components = new ArrayList<ScenarioComponent>();
        this.duration = cycleDuration;
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

    /** {@inheritDoc}
     * <p>The scenario will be run in loops until the target date
     * is reached. At each iteration of the loop, the target date of
     * the iteration will be set according to the cycle duration set at
     * construction.</p>
     */
    public ScenarioState[] updateStates(final ScenarioState[] origins, final AbsoluteDate target)
        throws OrekitException {

        ScenarioState[] states = origins.clone();
        AbsoluteDate iterationTarget;
        do {

            // set target date for iteration using cycle duration
            iterationTarget = states[0].getEstimatedStartState().getDate().shiftedBy(duration);

            // run all components of the scenario in order
            for (final ScenarioComponent component : components) {
                states = component.updateStates(states, iterationTarget);
            }

            // prepare next cycle
            for (int i = 0; i < states.length; ++i) {
                states[i] = states[i].updateCyclesNumber(states[i].getCyclesNumber() + 1);
                states[i] = states[i].updateRealStartState(states[i].getRealEndState());
                states[i] = states[i].updateEstimatedStartState(null);
                states[i] = states[i].updateRealEndState(null);
                states[i] = states[i].updateTheoreticalManeuvers(new ArrayList<ImpulseManeuver>());
                states[i] = states[i].updatePerformedManeuvers(new ArrayList<ImpulseManeuver>());
            }

        } while (iterationTarget.compareTo(target) < 0);

        return states;

    }

}
