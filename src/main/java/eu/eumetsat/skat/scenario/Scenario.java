/* Copyright 2011 Eumetsat */

package eu.eumetsat.skat.scenario;

import java.util.ArrayList;
import java.util.List;

import org.orekit.errors.OrekitException;
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
    public ScenarioState apply(final ScenarioState origin, final AbsoluteDate target)
        throws OrekitException {

        ScenarioState state = origin;
        AbsoluteDate iterationTarget;
        do {

            // set target date for iteration using cycle duration
            iterationTarget = state.getEstimatedState().getDate().shiftedBy(duration);

            // run all components of the scenario in order
            for (final ScenarioComponent component : components) {
                state = component.apply(state, iterationTarget);
            }

        } while (iterationTarget.compareTo(target) < 0);

        return state;

    }

}
