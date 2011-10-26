/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.scenario;

import java.util.ArrayList;
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

    /** Cycles number. */
    private final int cyclesNumber;

    /** Number of performed in-plane maneuvers. */
    private final int inPlane;

    /** Total dV in-plane maneuvers. */
    private final double inPlaneDV;

    /** Number of performed out-of-plane maneuvers. */
    private final int outOfPlane;

    /** Total dV out-of-plane maneuvers. */
    private final double outOfPlaneDV;

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

    /** Ephemeris throughout cycle. */
    private final BoundedPropagator ephemeris;

    /** Theoretical maneuvers. */
    private final List<ScheduledManeuver> theoreticalManeuvers;

    /** Performed maneuvers. */
    private final List<ScheduledManeuver> performedManeuvers;

    /** complete constructor.
     * @param name spacecraft name
     * @param cyclesNumber cycle number
     * @param inplane number of performed in-plane maneuvers
     * @param inPlaneDV total dV in-plane maneuvers
     * @param outOfPlane number of performed out-of-plane maneuvers
     * @param outOfPlaneDV total dV out-of-plane maneuvers
     * @param ascendingNodeDate date of previous ascending node
     * @param ascendingNodesSolarTime solar time at previous ascending node
     * @param descendingNodeDate date of previous descending node
     * @param descendingNodesSolarTime solar time at previous descending node
     * @param realStartState real state at cycle start
     * @param estimatedStartState estimated state at cycle start
     * @param realEndState real state at cycle end
     * @param ephemeris ephemeris throughout cycle
     * @param theoreticalManeuvers list of scheduled theoretical maneuvers
     * @param performedManeuvers list of performed maneuvers
     */
    private ScenarioState(final String name, final int cyclesNumber,
                          final int inPlane, final double inPlaneDV,
                          final int outOfPlane, final double outOfPlaneDV,
                          final AbsoluteDate ascendingNodeDate,
                          final double ascendingNodesSolarTime,
                          final AbsoluteDate descendingNodeDate,
                          final double descendingNodesSolarTime,
                          final SpacecraftState realStartState,
                          final SpacecraftState estimatedStartState,
                          final SpacecraftState realEndState,
                          final BoundedPropagator ephemeris,
                          final List<ScheduledManeuver> theoreticalManeuvers,
                          final List<ScheduledManeuver> performedManeuvers) {
        this.name                     = name;
        this.cyclesNumber             = cyclesNumber;
        this.inPlane                  = inPlane;
        this.inPlaneDV                = inPlaneDV;
        this.outOfPlane               = outOfPlane;
        this.outOfPlaneDV             = outOfPlaneDV;
        this.ascendingNodeDate        = ascendingNodeDate;
        this.ascendingNodesSolarTime  = ascendingNodesSolarTime;
        this.descendingNodeDate       = descendingNodeDate;
        this.descendingNodesSolarTime = descendingNodesSolarTime;
        this.realStartState           = realStartState;
        this.estimatedStartState      = estimatedStartState;
        this.realEndState             = realEndState;
        this.ephemeris                = ephemeris;
        this.theoreticalManeuvers     = theoreticalManeuvers;
        this.performedManeuvers       = performedManeuvers;
    }

    /** Simple constructor.
     * @param name spacecraft name
     * @param cyclesNumber cycles number
     * @param realState real state
     */
    public ScenarioState(final String name, final int cyclesNumber, final SpacecraftState realState) {
        this(name, cyclesNumber,
             0, 0.0, 0, 0.0,
             AbsoluteDate.PAST_INFINITY, Double.NaN,
             AbsoluteDate.PAST_INFINITY, Double.NaN,
             realState, null, null, null,
             new ArrayList<ScheduledManeuver>(),
             new ArrayList<ScheduledManeuver>());
    }

    /** Get the spacecraft name.
     * @return spacecraft name
     */
    public String getName() {
        return name;
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
        return new ScenarioState(name, cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 ephemeris, theoreticalManeuvers, performedManeuvers);
    }

    /** Get the number of performed in-plane maneuvers.
     * @return number of performed in-plane maneuvers
     */
    public int getInPlane() {
        return inPlane;
    }

    /** Get the total dV in-plane maneuvers.
     * @return total dV in-plane maneuvers
     */
    public double getInPlaneDV() {
        return inPlaneDV;
    }

    /** Update the in-plane maneuvers.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param number number of in-plane maneuvers
     * @param dV total DV for in-plane maneuvers
     * @return updated state
     */
    public ScenarioState updateInPlaneManeuvers(final int number, final double dv) {
        return new ScenarioState(name, cyclesNumber,
                                 number, dv, outOfPlane, outOfPlaneDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 ephemeris, theoreticalManeuvers, performedManeuvers);
    }

    /** Get the number of performed out-of-plane maneuvers.
     * @return number of performed out-of-plane maneuvers
     */
    public int getOutOfPlane() {
        return outOfPlane;
    }

    /** Get the total dV out-of-plane maneuvers.
     * @return total dV out-of-plane maneuvers
     */
    public double getOutOfPlaneDV() {
        return outOfPlaneDV;
    }

    /** Update the out-of-plane maneuvers.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param number number of out-of-plane maneuvers
     * @param dV total DV for iout-of-plane maneuvers
     * @return updated state
     */
    public ScenarioState updateOutOfPlaneManeuvers(final int number, final double dv) {
        return new ScenarioState(name, cyclesNumber,
                                 inPlane, inPlaneDV, number, dv,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 ephemeris, theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(name, cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 date, solarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 ephemeris, theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(name, cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 date, solarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 ephemeris, theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(name, cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 state, estimatedStartState, realEndState,
                                 ephemeris, theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(name, cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 ephemeris, theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(name, cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, state,
                                 ephemeris, theoreticalManeuvers, performedManeuvers);
    }

    /** Get the ephemeris throughout cycle.
     * @return ephemeris throughout cycle
     */
    public BoundedPropagator getEphemeris() {
        return ephemeris;
    }

    /** Update the ephemeris throughout cycle.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param ephemeris throughout cycle
     * @return updated state
     */
    public ScenarioState updateEphemeris(final BoundedPropagator ephemeris) {
        return new ScenarioState(name, cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 ephemeris, theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(name, cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 ephemeris, maneuvers, performedManeuvers);
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
        return new ScenarioState(name, cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realStartState, estimatedStartState, realEndState,
                                 ephemeris, theoreticalManeuvers, performedManeuvers);
    }

}
