/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.scenario;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.orekit.bodies.BodyShape;
import org.orekit.frames.Frame;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.strategies.ScheduledManeuver;

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
 * Instances of this class are guaranteed to be immutable
 * </p>
 * @author Luc Maisonobe
 */
public class ScenarioState {

    /** Spacecraft name. */
    private final String name;

    /** Inertial frame. */
    private final Frame inertialFrame;

    /** Earth model. */
    private final BodyShape earth;

    /** Spacecraft mass at Begin Of Life. */
    private final double bolMass;

    /** Cycles number. */
    private final int cyclesNumber;

    /** Target cycle end. */
    private final AbsoluteDate targetCycleEnd;

    /** Real state. */
    private final SpacecraftState realState;

    /** Estimated state. */
    private final SpacecraftState estimatedState;

    /** Performed ephemeris throughout cycle. */
    private final BoundedPropagator performedEphemeris;

    /** Maneuvers. */
    private final List<ScheduledManeuver> maneuvers;

    /** Maneuvers statistics. */
    private final Map<String, ManeuverStats> maneuversStats;

    /** complete constructor.
     * @param name spacecraft name
     * @param inertialFrame inertial frame
     * @param earth Earth model
     * @param bolMass spacecraft mass at Begin Of Life
     * @param cyclesNumber cycle number
     * @param targetCycleEnd target cycle end
     * @param realState real state at cycle start
     * @param estimatedState estimated state at cycle start
     * @param performedEphemeris performed ephemeris throughout cycle
     * @param maneuvers list of maneuvers
     * @param inplane number of performed in-plane maneuvers
     */
    private ScenarioState(final String name, final Frame inertialFrame, BodyShape earth,
                          final double bolMass, final int cyclesNumber,
                          final AbsoluteDate targetCycleEnd,
                          final SpacecraftState realState,
                          final SpacecraftState estimatedState,
                          final BoundedPropagator performedEphemeris,
                          final List<ScheduledManeuver> maneuvers,
                          final Map<String, ManeuverStats> maneuversStats) {
        this.name               = name;
        this.inertialFrame      = inertialFrame;
        this.earth              = earth;
        this.bolMass            = bolMass;
        this.cyclesNumber       = cyclesNumber;
        this.targetCycleEnd     = targetCycleEnd;
        this.realState          = realState;
        this.estimatedState     = estimatedState;
        this.performedEphemeris = performedEphemeris;
        this.maneuvers          = maneuvers;
        this.maneuversStats     = maneuversStats;
    }

    /** Simple constructor.
     * @param name spacecraft name
     * @param inertialFrame inertial frame
     * @param earth Earth model
     * @param bolMass spacecraft mass at Begin Of Life
     * @param cyclesNumber cycles number
     * @param realState real state
     */
    public ScenarioState(final String name, final Frame inertialFrame, BodyShape earth,
                         final double bolMass, final int cyclesNumber,
                         final SpacecraftState realState) {
        this(name, inertialFrame, earth, bolMass, cyclesNumber,
             realState.getDate().shiftedBy(Constants.JULIAN_CENTURY),
             realState, null, null, null, new HashMap<String, ManeuverStats>());
    }

    /** Get the spacecraft name.
     * @return spacecraft name
     */
    public String getName() {
        return name;
    }

    /** Get the inertial frame.
     * @return inertial frame
     */
    public Frame getInertialFrame() {
        return inertialFrame;
    }

    /** Get the Earth model.
     * @return Earth model
     */
    public BodyShape getEarth() {
        return earth;
    }

    /** Get the spacecraft mass at Begin Of Life.
     * @return spacecraft mass at Begin Of Life
     */
    public double getBOLMass() {
        return bolMass;
    }

    /** Get the cycles number.
     * @return cycles number
     */
    public int getCyclesNumber() {
        return cyclesNumber;
    }

    /** Update the cycles number.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param cyclesNumber cycles number
     * @return updated state
     */
    public ScenarioState updateCyclesNumber(final int cyclesNumber) {
        return new ScenarioState(name, inertialFrame, earth, bolMass, cyclesNumber,
                                 targetCycleEnd, realState, estimatedState,
                                 performedEphemeris, maneuvers, maneuversStats);
    }

    /** Get the target cycle end.
     * @return target cycle end
     */
    public AbsoluteDate getTargetCycleEnd() {
        return targetCycleEnd;
    }

    /** Update the target cycle end.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param targetCycleEnd target cycle end
     * @return updated state
     */
    public ScenarioState updateTargetCycleEnd(final AbsoluteDate targetCycleEnd) {
        return new ScenarioState(name, inertialFrame, earth, bolMass, cyclesNumber,
                                 targetCycleEnd, realState, estimatedState,
                                 performedEphemeris, maneuvers, maneuversStats);
    }

    /** Get the name of all maneuvers.
     * @return names of all maneuvers
     */
    public Set<String> getManeuversNames() {
        return maneuversStats.keySet();
    }

    /** Get the number of performed maneuvers.
     * @param name name of the selected maneuver
     * @return number of performed maneuvers
     */
    public int getManeuversNumber(final String name) {
        return maneuversStats.get(name).getNumber();
    }

    /** Get the cycle dV maneuvers.
     * @param name name of the selected maneuver
     * @return cycle dV maneuvers
     */
    public double getManeuversCycleDV(final String name) {
        return maneuversStats.get(name).getCycleDV();
    }

    /** Get the total dV maneuvers.
     * @param name name of the selected maneuver
     * @return total dV maneuvers
     */
    public double getManeuversTotalDV(final String name) {
        return maneuversStats.get(name).getTotalDV();
    }

    /** Update a maneuver statistics.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param maneuverName name of the maneuver
     * @param number number of named maneuvers
     * @param cycleDV cycle DV for the named maneuver
     * @param totalDV total DV for the named maneuver
     * @return updated state
     */
    public ScenarioState updateManeuverStats(final String maneuverName, final int number,
                                             final double cycleDV, final double totalDV) {
        final HashMap<String, ManeuverStats> updatedMap =
                new HashMap<String, ScenarioState.ManeuverStats>(maneuversStats);
        updatedMap.put(maneuverName, new ManeuverStats(number, cycleDV, totalDV));
        return new ScenarioState(name, inertialFrame, earth, bolMass, cyclesNumber,
                                 targetCycleEnd, realState, estimatedState,
                                 performedEphemeris, maneuvers, updatedMap);
    }

    /** Get the real state.
     * @return real state
     */
    public SpacecraftState getRealState() {
        return realState;
    }

    /** Update the real state.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param state real state
     * @return updated state
     */
    public ScenarioState updateRealState(final SpacecraftState state) {
        return new ScenarioState(name, inertialFrame, earth, bolMass, cyclesNumber,
                                 targetCycleEnd, state, estimatedState,
                                 performedEphemeris, maneuvers, maneuversStats);
    }

    /** Get the estimated state.
     * @return estimated state
     */
    public SpacecraftState getEstimatedState() {
        return estimatedState;
    }

    /** Update the estimated state.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param state estimated state
     * @return updated state
     */
    public ScenarioState updateEstimatedState(final SpacecraftState state) {
        return new ScenarioState(name, inertialFrame, earth, bolMass, cyclesNumber,
                                 targetCycleEnd, realState, state,
                                 performedEphemeris, maneuvers, maneuversStats);
    }

    /** Get the performed ephemeris throughout cycle.
     * @return performed ephemeris throughout cycle
     */
    public BoundedPropagator getPerformedEphemeris() {
        return performedEphemeris;
    }

    /** Update the performed ephemeris throughout cycle.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param ephemeris performed ephemeris throughout cycle
     * @return updated state
     */
    public ScenarioState updatePerformedEphemeris(final BoundedPropagator ephemeris) {
        return new ScenarioState(name, inertialFrame, earth, bolMass, cyclesNumber,
                                 targetCycleEnd, realState, estimatedState,
                                 ephemeris, maneuvers, maneuversStats);
    }

    /** Get the scheduled maneuvers.
     * @return scheduled maneuvers
     */
    public List<ScheduledManeuver> getManeuvers() {
        return maneuvers;
    }

    /** Update the maneuvers.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param maneuvers maneuvers
     * @return updated state
     */
    public ScenarioState updateManeuvers(final List<ScheduledManeuver> maneuvers) {
        return new ScenarioState(name, inertialFrame, earth, bolMass, cyclesNumber,
                                 targetCycleEnd, realState, estimatedState,
                                 performedEphemeris, maneuvers, maneuversStats);
    }

    /** Inner class for maneuvers statistics. */
    private static class ManeuverStats {

        /** Number of maneuvers performed. */
        private final int number;

        /** Velocity increment for the current cycle. */
        private final double cycleDV;

        /** Velocity increment since simulation start. */
        private final double totalDV;

        /** Simple constructor.
         * @param number number of maneuvers performed
         * @param cycleDV velocity increment for the current cycle
         * @param totalDV velocity increment since simulation start
         */
        public ManeuverStats(final int number, final double cycleDV, final double totalDV) {
            this.number  = number;
            this.cycleDV = cycleDV;
            this.totalDV = totalDV;
        }

        /** Get the number of maneuvers performed.
         * @return number of maneuvers performed
         */
        public int getNumber() {
            return number;
        }

        /** Get the velocity increment for the current cycle.
         * @return velocity increment for the current cycle
         */
        public double getCycleDV() {
            return cycleDV;
        }

        /** Get the velocity increment since simulation start.
         * @return velocity increment since simulation start
         */
        public double getTotalDV() {
            return totalDV;
        }

    }

}
