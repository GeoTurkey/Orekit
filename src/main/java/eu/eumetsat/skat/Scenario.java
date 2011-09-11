/* Copyright 2011 Eumetsat */

package eu.eumetsat.skat;

import java.util.ArrayList;
import java.util.List;

/** Station-Keeping scenario.
 * <p>
 * A station-keeping scenario is a simple list of cycle components
 * that are run one after the other at each cycle, and then the cycle
 * is repeated.
 * </p>
 */
public class Scenario {

    /** Cycle components. */
    private final List<CycleComponent> components;

    /** Maximal cycle duration. */
    private final double cycleDuration;

    /** Simple constructor.
     * <p>
     * Create an empty scenario
     * </p>
     * @param cycleDuration cycle duration (s)
     */
    public Scenario(final double cycleDuration) {
        components = new ArrayList<CycleComponent>();
        this.cycleDuration = cycleDuration;
    }

    /** Add a cycle component.
     * <p>
     * Cycle components must be added in simulation order.
     * </p>
     * @param component cycle component
     */
    public void addComponent(final CycleComponent component) {
        components.add(component);
    }

}
