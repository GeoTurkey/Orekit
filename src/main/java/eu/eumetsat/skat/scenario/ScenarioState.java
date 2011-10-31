/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.scenario;

import java.util.List;

import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;

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

    /** Spacecraft name. */
    private final String name;

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

    /** Date of previous ascending node. */
    private final AbsoluteDate ascendingNodeDate;

    /** Solar time at previous ascending node. */
    private final double ascendingNodesSolarTime;

    /** Date of previous descending node. */
    private final AbsoluteDate descendingNodeDate;

    /** Solar time at previous descending node. */
    private final double descendingNodesSolarTime;

    /** Real state at cycle start. */
    private final SpacecraftState realStartState;

    /** Estimated state at cycle start. */
    private final SpacecraftState estimatedStartState;

    /** Real state at cycle end. */
    private final SpacecraftState realEndState;

    /** Theoretical ephemeris throughout cycle. */
    private final BoundedPropagator theoreticalEphemeris;

    /** Performed ephemeris throughout cycle. */
    private final BoundedPropagator performedEphemeris;

    /** Theoretical maneuvers. */
    private final List<ScheduledManeuver> theoreticalManeuvers;

    /** Performed maneuvers. */
    private final List<ScheduledManeuver> performedManeuvers;

    /** complete constructor.
     * @param name spacecraft name
     * @param bolMass spacecraft mass at Begin Of Life
     * @param cyclesNumber cycle number
     * @param inplane number of performed in-plane maneuvers
     * @param inPlaneTotalDV total dV in-plane maneuvers
     * @param outOfPlane number of performed out-of-plane maneuvers
     * @param outOfPlaneTotalDV total dV out-of-plane maneuvers
     * @param ascendingNodeDate date of previous ascending node
     * @param ascendingNodesSolarTime solar time at previous ascending node
     * @param descendingNodeDate date of previous descending node
     * @param descendingNodesSolarTime solar time at previous descending node
     * @param realStartState real state at cycle start
     * @param estimatedStartState estimated state at cycle start
     * @param realEndState real state at cycle end
     * @param theoreticalEphemeris theoretical ephemeris throughout cycle
     * @param performedEphemeris performed ephemeris throughout cycle
     * @param theoreticalManeuvers list of scheduled theoretical maneuvers
     * @param performedManeuvers list of performed maneuvers
     */
    private ScenarioState(final String name, final double bolMass, final int cyclesNumber,
                          final int inPlane, final double inPlaneCycleDV, final double inPlaneTotalDV,
                          final int outOfPlane, final double outOfPlaneCycleDV, final double outOfPlaneTotalDV,
                          final AbsoluteDate ascendingNodeDate,
                          final double ascendingNodesSolarTime,
                          final AbsoluteDate descendingNodeDate,
                          final double descendingNodesSolarTime,
                          final SpacecraftState realStartState,
                          final SpacecraftState estimatedStartState,
                          final SpacecraftState realEndState,
                          final BoundedPropagator theoreticalEphemeris,
                          final BoundedPropagator performedEphemeris,
                          final List<ScheduledManeuver> theoreticalManeuvers,
                          final List<ScheduledManeuver> performedManeuvers) {
        this.name                     = name;
        this.bolMass                  = bolMass;
        this.cyclesNumber             = cyclesNumber;
        this.inPlaneManeuvers         = inPlane;
        this.inPlaneCycleDV           = inPlaneCycleDV;
        this.inPlaneTotalDV           = inPlaneTotalDV;
        this.outOfPlaneManeuvers      = outOfPlane;
        this.outOfPlaneCycleDV        = outOfPlaneCycleDV;
        this.outOfPlaneTotalDV        = outOfPlaneTotalDV;
        this.ascendingNodeDate        = ascendingNodeDate;
        this.ascendingNodesSolarTime  = ascendingNodesSolarTime;
        this.descendingNodeDate       = descendingNodeDate;
        this.descendingNodesSolarTime = descendingNodesSolarTime;
        this.realStartState           = realStartState;
        this.estimatedStartState      = estimatedStartState;
        this.realEndState             = realEndState;
        this.theoreticalEphemeris     = theoreticalEphemeris;
        this.performedEphemeris       = performedEphemeris;
        this.theoreticalManeuvers     = theoreticalManeuvers;
        this.performedManeuvers       = performedManeuvers;
    }

    /** Simple constructor.
     * @param name spacecraft name
     * @param bolMass spacecraft mass at Begin Of Life
     * @param cyclesNumber cycles number
     * @param realState real state
     */
    public ScenarioState(final String name, final double bolMass,
                         final int cyclesNumber, final SpacecraftState realState) {
        this(name, bolMass, cyclesNumber,
             0, 0.0, 0.0, 0, 0.0, 0.0,
             AbsoluteDate.PAST_INFINITY, Double.NaN,
             AbsoluteDate.PAST_INFINITY, Double.NaN,
             realState, null, null, null, null, null, null);
    }

    /** Get the spacecraft name.
     * @return spacecraft name
     */
    public String getName() {
        return name;
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
        return new ScenarioState(name, bolMass, cyclesNumber,
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 theoreticalEphemeris, performedEphemeris,
                                 theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(name, bolMass, cyclesNumber,
                                 number, cycleDV, totalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 theoreticalEphemeris, performedEphemeris,
                                 theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(name, bolMass, cyclesNumber,
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 number, cycleDV, totalDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 theoreticalEphemeris, performedEphemeris,
                                 theoreticalManeuvers, performedManeuvers);
    }

    /** Get the date of previous ascending node.
     * @return date of previous ascending node
     */
    public AbsoluteDate getAscendingNodeDate() {
        return ascendingNodeDate;
    }

    /** Get the solar time at previous ascending node.
     * @return solar time at previous ascending node
     */
    public double getAscendingNodesSolarTime() {
        return ascendingNodesSolarTime;
    }

    /** Update the ascending node crossing.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param date date of ascending node crossing
     * @param solarTime solar time at ascending node crossing
     * @return updated state
     */
    public ScenarioState updateAscendingNodeCrossing(final AbsoluteDate date,
                                                     final double solarTime) {
        return new ScenarioState(name, bolMass, cyclesNumber,
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 date, solarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 theoreticalEphemeris, performedEphemeris,
                                 theoreticalManeuvers, performedManeuvers);
    }

    /** Get the date of previous descending node.
     * @return date of previous descending node
     */
    public AbsoluteDate getDescendingNodeDate() {
        return descendingNodeDate;
    }

    /** Get the solar time at previous descending node.
     * @return solar time at previous descending node
     */
    public double getDescendingNodesSolarTime() {
        return descendingNodesSolarTime;
    }

    /** Update the descending node crossing.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param date date of descending node crossing
     * @param solarTime solar time at descending node crossing
     * @return updated state
     */
    public ScenarioState updateDescendingNodeCrossing(final AbsoluteDate date,
                                                      final double solarTime) {
        return new ScenarioState(name, bolMass, cyclesNumber,
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 date, solarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 theoreticalEphemeris, performedEphemeris,
                                 theoreticalManeuvers, performedManeuvers);
    }

    /** Get the real state at cycle start.
     * @return real state at cycle start
     */
    public SpacecraftState getRealStartState() {
        return realStartState;
    }

    /** Update the real state at cycle start.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param state real state at cycle start
     * @return updated state
     */
    public ScenarioState updateRealStartState(final SpacecraftState state) {
        return new ScenarioState(name, bolMass, cyclesNumber,
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 state, estimatedStartState, realEndState,
                                 theoreticalEphemeris, performedEphemeris,
                                 theoreticalManeuvers, performedManeuvers);
    }

    /** Get the estimated state at cycle start.
     * @return estimated state at cycle start
     */
    public SpacecraftState getEstimatedStartState() {
        return estimatedStartState;
    }

    /** Update the estimated state at cycle start.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param state estimated state at cycle start
     * @return updated state
     */
    public ScenarioState updateEstimatedStartState(final SpacecraftState state) {
        return new ScenarioState(name, bolMass, cyclesNumber,
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, state, realEndState,
                                 theoreticalEphemeris, performedEphemeris,
                                 theoreticalManeuvers, performedManeuvers);
    }

    /** Get the real state at cycle end.
     * @return real state at cycle end
     */
    public SpacecraftState getRealEndState() {
        return realEndState;
    }

    /** Update the real state at cycle end.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param state real state at cycle end
     * @return updated state
     */
    public ScenarioState updateRealEndState(final SpacecraftState state) {
        return new ScenarioState(name, bolMass, cyclesNumber,
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, state,
                                 theoreticalEphemeris, performedEphemeris,
                                 theoreticalManeuvers, performedManeuvers);
    }

    /** Get the theoretical ephemeris throughout cycle.
     * @return theoretical ephemeris throughout cycle
     */
    public BoundedPropagator getTheoreticalEphemeris() {
        return theoreticalEphemeris;
    }

    /** Update the theoretical ephemeris throughout cycle.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param theoreticalEphemeris theoretical ephemeris throughout cycle
     * @return updated state
     */
    public ScenarioState updateTheoreticalEphemeris(final BoundedPropagator theoreticalEphemeris) {
        return new ScenarioState(name, bolMass, cyclesNumber,
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 theoreticalEphemeris, performedEphemeris,
                                 theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(name, bolMass, cyclesNumber,
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 theoreticalEphemeris, ephemeris,
                                 theoreticalManeuvers, performedManeuvers);
    }

    /** Get the scheduled theoretical maneuvers.
     * @return scheduled theoretical maneuvers
     */
    public List<ScheduledManeuver> getTheoreticalManeuvers() {
        return theoreticalManeuvers;
    }

    /** Update the theoretical maneuvers.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param maneuvers theoretical maneuvers
     * @return updated state
     */
    public ScenarioState updateTheoreticalManeuvers(final List<ScheduledManeuver> maneuvers) {
        return new ScenarioState(name, bolMass, cyclesNumber,
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 theoreticalEphemeris, performedEphemeris,
                                 maneuvers, performedManeuvers);
    }

    /** Get the performed maneuvers.
     * @return performed maneuvers
     */
    public List<ScheduledManeuver> getPerformedManeuvers() {
        return performedManeuvers;
    }

    /** Update the performed maneuvers.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param maneuvers performed maneuvers
     * @return updated state
     */
    public ScenarioState updatePerformedManeuvers(final List<ScheduledManeuver> maneuvers) {
        return new ScenarioState(name, bolMass, cyclesNumber,
                                 inPlaneManeuvers, inPlaneCycleDV, inPlaneTotalDV,
                                 outOfPlaneManeuvers, outOfPlaneCycleDV, outOfPlaneTotalDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 theoreticalEphemeris, performedEphemeris,
                                 theoreticalManeuvers, maneuvers);
    }

}
