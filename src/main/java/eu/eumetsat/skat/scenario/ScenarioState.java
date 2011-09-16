/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.scenario;

import java.util.List;

import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.SpacecraftState;

/**
 * Container for scenario states, containing real and estimated spacecraft
 * state as well as theoreticalManeuvers.
 * <p>
 * The real state in the pair represents the true state of the satellite
 * as simulated using appropriate propagation models, using the
 * maneuvers really performed. The estimated state represents what the
 * ground system thinks the real state is, after orbit determination
 * and applying theoretical maneuver without errors.
 * </p>
 * <p>
 * There are also two types of maneuvers, the theoretical ones computed
 * by the station-keeping ground system, and the ones really performed by
 * the spacecraft.
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

    /** Theoretical maneuvers. */
    private final List<ImpulseManeuver> theoreticalManeuvers;

    /** Performed maneuvers. */
    private final List<ImpulseManeuver> performedManeuvers;

    /** Simple constructor.
     * @param realState real state
     * @param estimatedState estimated state
     * @param theoreticalManeuvers list of scheduled theoretical maneuvers
     * @param performedManeuvers list of performed maneuvers
     */
    public ScenarioState(final SpacecraftState realState,
                         final SpacecraftState estimatedState,
                         final List<ImpulseManeuver> theoreticalManeuvers,
                         final List<ImpulseManeuver> performedManeuvers) {
        this.realState            = realState;
        this.estimatedState       = estimatedState;
        this.theoreticalManeuvers = theoreticalManeuvers;
        this.performedManeuvers   = performedManeuvers;
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

    /** Get the scheduled theoretical maneuvers.
     * @return scheduled theoretical maneuvers
     */
    public List<ImpulseManeuver> getTheoreticalManeuvers() {
        return theoreticalManeuvers;
    }

    /** Get the performed maneuvers.
     * @return performed maneuvers
     */
    public List<ImpulseManeuver> getPerformedManeuvers() {
        return performedManeuvers;
    }

}
