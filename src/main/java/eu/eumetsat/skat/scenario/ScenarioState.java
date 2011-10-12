/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.scenario;

import java.util.ArrayList;
import java.util.List;

import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;

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

    /** Mass consumption. */
    private final double massConsumption;

    /** Date of previous ascending node. */
    private final AbsoluteDate ascendingNodeDate;

    /** Solar time at previous ascending node. */
    private final double ascendingNodesSolarTime;

    /** Date of previous descending node. */
    private final AbsoluteDate descendingNodeDate;

    /** Solar time at previous descending node. */
    private final double descendingNodesSolarTime;

    /** Real state. */
    private final SpacecraftState realState;

    /** Estimated state. */
    private final SpacecraftState estimatedState;

    /** Theoretical maneuvers. */
    private final List<ImpulseManeuver> theoreticalManeuvers;

    /** Performed maneuvers. */
    private final List<ImpulseManeuver> performedManeuvers;

    /** complete constructor.
     * @param cyclesNumber cycle number
     * @param inplane number of performed in-plane maneuvers
     * @param inPlaneDV total dV in-plane maneuvers
     * @param outOfPlane number of performed out-of-plane maneuvers
     * @param outOfPlaneDV total dV out-of-plane maneuvers
     * @param massConsumption mass consumption
     * @param ascendingNodeDate date of previous ascending node
     * @param ascendingNodesSolarTime solar time at previous ascending node
     * @param descendingNodeDate date of previous descending node
     * @param descendingNodesSolarTime solar time at previous descending node
     * @param realState real state
     * @param estimatedState estimated state
     * @param theoreticalManeuvers list of scheduled theoretical maneuvers
     * @param performedManeuvers list of performed maneuvers
     */
    private ScenarioState(final int cyclesNumber,
                          final int inPlane, final double inPlaneDV,
                          final int outOfPlane, final double outOfPlaneDV,
                          final double massConsumption,
                          final AbsoluteDate ascendingNodeDate,
                          final double ascendingNodesSolarTime,
                          final AbsoluteDate descendingNodeDate,
                          final double descendingNodesSolarTime,
                          final SpacecraftState realState,
                          final SpacecraftState estimatedState,
                          final List<ImpulseManeuver> theoreticalManeuvers,
                          final List<ImpulseManeuver> performedManeuvers) {
        this.cyclesNumber             = cyclesNumber;
        this.inPlane                  = inPlane;
        this.inPlaneDV                = inPlaneDV;
        this.outOfPlane               = outOfPlane;
        this.outOfPlaneDV             = outOfPlaneDV;
        this.massConsumption          = massConsumption;
        this.ascendingNodeDate        = ascendingNodeDate;
        this.ascendingNodesSolarTime  = ascendingNodesSolarTime;
        this.descendingNodeDate       = descendingNodeDate;
        this.descendingNodesSolarTime = descendingNodesSolarTime;
        this.realState                = realState;
        this.estimatedState           = estimatedState;
        this.theoreticalManeuvers     = theoreticalManeuvers;
        this.performedManeuvers       = performedManeuvers;
    }

    /** Simple constructor.
     * @param cyclesNumber cycles number
     * @param realState real state
     */
    public ScenarioState(final int cyclesNumber, final SpacecraftState realState) {
        this(cyclesNumber,
             0, 0.0, 0, 0.0,
             0.0,
             AbsoluteDate.PAST_INFINITY, Double.NaN,
             AbsoluteDate.PAST_INFINITY, Double.NaN,
             realState, realState,
             new ArrayList<ImpulseManeuver>(),
             new ArrayList<ImpulseManeuver>());
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
        return new ScenarioState(cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 massConsumption,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realState, estimatedState,
                                 theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(cyclesNumber,
                                 number, dv, outOfPlane, outOfPlaneDV,
                                 massConsumption,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realState, estimatedState,
                                 theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(cyclesNumber,
                                 inPlane, inPlaneDV, number, dv,
                                 massConsumption,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realState, estimatedState,
                                 theoreticalManeuvers, performedManeuvers);
    }

    /** get the mass consumption.
     * @return mass consumption
     */
    public double getMassConsumption() {
        return massConsumption;
    }

    /** Update the mass consumption.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param massConsumption mass consumption
     * @return updated state
     */
    public ScenarioState updateMassConsumption(final double massConsumption) {
        return new ScenarioState(cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 massConsumption,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realState, estimatedState,
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
        return new ScenarioState(cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 massConsumption,
                                 date, solarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realState, estimatedState,
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
        return new ScenarioState(cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 massConsumption,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 date, solarTime,
                                 realState, estimatedState,
                                 theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 massConsumption,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 state, estimatedState,
                                 theoreticalManeuvers, performedManeuvers);
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
        return new ScenarioState(cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 massConsumption,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realState, estimatedState,
                                 theoreticalManeuvers, performedManeuvers);
    }

    /** Get the scheduled theoretical maneuvers.
     * @return scheduled theoretical maneuvers
     */
    public List<ImpulseManeuver> getTheoreticalManeuvers() {
        return theoreticalManeuvers;
    }

    /** Update the theoretical maneuvers.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param maneuvers theoretical maneuvers
     * @return updated state
     */
    public ScenarioState updateTheoreticalManeuvers(final List<ImpulseManeuver> maneuvers) {
        return new ScenarioState(cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 massConsumption,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realState, estimatedState,
                                 maneuvers, performedManeuvers);
    }

    /** Get the performed maneuvers.
     * @return performed maneuvers
     */
    public List<ImpulseManeuver> getPerformedManeuvers() {
        return performedManeuvers;
    }

    /** Update the performed maneuvers.
     * <p>
     * The instance is not changed, a new instance is created
     * </p>
     * @param maneuvers performed maneuvers
     * @return updated state
     */
    public ScenarioState updatePerformedManeuvers(final List<ImpulseManeuver> maneuvers) {
        return new ScenarioState(cyclesNumber,
                                 inPlane, inPlaneDV, outOfPlane, outOfPlaneDV,
                                 massConsumption,
                                 ascendingNodeDate, ascendingNodesSolarTime,
                                 descendingNodeDate, descendingNodesSolarTime,
                                 realState, estimatedState,
                                 theoreticalManeuvers, performedManeuvers);
    }

}
