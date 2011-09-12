/* Copyright 2011 Eumetsat */

package eu.eumetsat.skat.scenario;

import java.util.List;

import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.SpacecraftState;

/**
 * Container for scenario states, containing real and estimated spacecraft
 * state as well as maneuvers.
 * <p>
 * The real state in the pair represents the true state of the satellite
 * as simulated using appropriate propagation models, using the
 * maneuvers really performed. The estimated state represents what the
 * ground system thinks the real state is, after orbit determination
 * and applying theoretical maneuver without errors.
 * </p>
 * <p>
 * Instances of this class are guaranteed to be immutable
 * </p>
 * @author Luc Maisonobe
 */
public class ScenarioState {

    /** Real state. */
    private final SpacecraftState realState;

    /** Estimated state. */
    private final SpacecraftState estimatedState;

    /** Maneuvers. */
    private final List<ImpulseManeuver> maneuvers;

    /** Simple constructor.
     * @param realState real state
     * @param estimatedState estimated state
     * @param maneuvers list of scheduled maneuvers
     */
    public ScenarioState(final SpacecraftState realState,
                         final SpacecraftState estimatedState,
                         final List<ImpulseManeuver> maneuvers) {
        this.realState      = realState;
        this.estimatedState = estimatedState;
        this.maneuvers      = maneuvers;
    }

    /** Get the real state.
     * @return real state
     */
    public SpacecraftState getRealState() {
        return realState;
    }

    /** Get the estimated state.
     * @return estimated state
     */
    public SpacecraftState getEstimatedState() {
        return estimatedState;
    }

    /** Get the scheduled maneuvers.
     * @return scheduled maneuvers
     */
    public List<ImpulseManeuver> getManeuvers() {
        return maneuvers;
    }

}
