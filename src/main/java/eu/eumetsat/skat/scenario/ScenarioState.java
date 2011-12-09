/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.scenario;

import java.util.List;

import org.orekit.bodies.BodyShape;
import org.orekit.frames.Frame;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.SpacecraftState;

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

    /** Number of performed in-plane maneuvers. */
    private final int inPlaneManeuvers;

    /** cycle dV in-plane maneuvers. */
    private final double inPlaneCycleDV;

    /** Total dV in-plane maneuvers. */
    private final double inPlaneTotalDV;

    /** Number of performed out-of-plane maneuvers. */
    private final int outOfPlaneManeuvers;

    /** Cycle dV out-of-plane maneuvers. */
    private final double outOfPlaneCycleDV;

    /** Total dV out-of-plane maneuvers. */
    private final double outOfPlaneTotalDV;

    /** Real state. */
    private final SpacecraftState realState;

    /** Estimated state. */
    private final SpacecraftState estimatedState;

    /** Performed ephemeris throughout cycle. */
    private final BoundedPropagator performedEphemeris;

    /** Maneuvers. */
    private final List<ScheduledManeuver> maneuvers;

    /** complete constructor.
     * @param name spacecraft name
     * @param inertialFrame inertial frame
     * @param earth Earth model
     * @param bolMass spacecraft mass at Begin Of Life
     * @param cyclesNumber cycle number
     * @param inPlaneTotalDV total dV in-plane maneuvers
     * @param outOfPlane number of performed out-of-plane maneuvers
     * @param outOfPlaneTotalDV total dV out-of-plane maneuvers
     * @param realState real state at cycle start
     * @param estimatedState estimated state at cycle start
     * @param performedEphemeris performed ephemeris throughout cycle
     * @param maneuvers list of maneuvers
     * @param inplane number of performed in-plane maneuvers
     */
    private ScenarioState(final String name, final Frame inertialFrame, BodyShape earth,
                          final double bolMass, final int cyclesNumber,
                          final int inPlane, final double inPlaneCycleDV, final double inPlaneTotalDV,
                          final int outOfPlane, final double outOfPlaneCycleDV, final double outOfPlaneTotalDV,
                          final SpacecraftState realState,
                          final SpacecraftState estimatedState,
                          final BoundedPropagator performedEphemeris,
                          final List<ScheduledManeuver> maneuvers) {
        this.name                     = name;
        this.inertialFrame            = inertialFrame;
        this.earth                    = earth;
        this.bolMass                  = bolMass;
        this.cyclesNumber             = cyclesNumber;
        this.inPlaneManeuvers         = inPlane;
        this.inPlaneCycleDV           = inPlaneCycleDV;
        this.inPlaneTotalDV           = inPlaneTotalDV;
        this.outOfPlaneManeuvers      = outOfPlane;
        this.outOfPlaneCycleDV        = outOfPlaneCycleDV;
        this.outOfPlaneTotalDV        = outOfPlaneTotalDV;
        this.realState                = realState;
        this.estimatedState           = estimatedState;
        this.performedEphemeris       = performedEphemeris;
        this.maneuvers                = maneuvers;
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
                         final double bolMass,
                         final int cyclesNumber, final SpacecraftState realState) {
        this(name, inertialFrame, earth, bolMass, cyclesNumber,
             0, 0.0, 0.0, 0, 0.0, 0.0,
             realState, null, null, null);
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
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 realState, estimatedState, performedEphemeris,
                                 maneuvers);
    }

    /** Get the number of performed in-plane maneuvers.
     * @return number of performed in-plane maneuvers
     */
    public int getInPlaneManeuvers() {
        return inPlaneManeuvers;
    }

    /** Get the cycle dV in-plane maneuvers.
     * @return cycle dV in-plane maneuvers
     */
    public double getInPlaneCycleDV() {
        return inPlaneCycleDV;
    }

    /** Get the total dV in-plane maneuvers.
     * @return total dV in-plane maneuvers
     */
    public double getInPlaneTotalDV() {
        return inPlaneTotalDV;
    }

    /** Update the in-plane maneuvers.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param number number of in-plane maneuvers
     * @param cycleDV cycle DV for in-plane maneuvers
     * @param totalDV total DV for in-plane maneuvers
     * @return updated state
     */
    public ScenarioState updateInPlaneManeuvers(final int number, final double cycleDV, final double totalDV) {
        return new ScenarioState(name, inertialFrame, earth, bolMass, cyclesNumber,
                                 number, cycleDV, totalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 realState, estimatedState, performedEphemeris,
                                 maneuvers);
    }

    /** Get the number of performed out-of-plane maneuvers.
     * @return number of performed out-of-plane maneuvers
     */
    public int getOutOfPlaneManeuvers() {
        return outOfPlaneManeuvers;
    }

    /** Get the cycle dV out-of-plane maneuvers.
     * @return cycle dV out-of-plane maneuvers
     */
    public double getOutOfPlaneCycleDV() {
        return outOfPlaneCycleDV;
    }

    /** Get the total dV out-of-plane maneuvers.
     * @return total dV out-of-plane maneuvers
     */
    public double getOutOfPlaneTotalDV() {
        return outOfPlaneTotalDV;
    }

    /** Update the out-of-plane maneuvers.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param number number of out-of-plane maneuvers
     * @param cycleDV cycle DV for out-of-plane maneuvers
     * @param totalDV total DV for out-of-plane maneuvers
     * @return updated state
     */
    public ScenarioState updateOutOfPlaneManeuvers(final int number, final double cycleDV, final double totalDV) {
        return new ScenarioState(name, inertialFrame, earth, bolMass, cyclesNumber,
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 number, cycleDV, totalDV,
                                 realState, estimatedState, performedEphemeris,
                                 maneuvers);
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
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 state, estimatedState, performedEphemeris,
                                 maneuvers);
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
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 realState, state, performedEphemeris,
                                 maneuvers);
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
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 realState, estimatedState, ephemeris,
                                 maneuvers);
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
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 realState, estimatedState, performedEphemeris,
                                 maneuvers);
    }

}
