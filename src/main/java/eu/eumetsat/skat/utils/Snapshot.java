/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.orekit.bodies.BodyShape;
import org.orekit.propagation.SpacecraftState;

/**
 * Container for simulation state snapshots, containing spacecraft states and maneuvers.
 * <p>
 * Instances of this class are guaranteed to be immutable
 * </p>
 * @author Luc Maisonobe
 */
public class Snapshot {

    /** State. */
    private final SpacecraftState state;

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

    /** Number of cycles. */
    private final int cyclesNumber;

    /** Earth model. */
    private final BodyShape earth;

    /** Simple constructor.
     * @param state state
     * @param inplane number of performed in-plane maneuvers
     * @param inPlaneDV total dV in-plane maneuvers
     * @param outOfPlane number of performed out-of-plane maneuvers
     * @param outOfPlaneDV total dV out-of-plane maneuvers
     * @param massConsumption mass consumption
     * @param cyclesNumber number of cycles
     * @param earth Earth model
     */
    public Snapshot(final SpacecraftState state,
                    final int inPlane, final double inPlaneDV,
                    final int outOfPlane, final double outOfPlaneDV,
                    final double massConsumption, final int cyclesNumber, final BodyShape earth) {
        this.state           = state;
        this.inPlane         = inPlane;
        this.inPlaneDV       = inPlaneDV;
        this.outOfPlane      = outOfPlane;
        this.outOfPlaneDV    = outOfPlaneDV;
        this.massConsumption = massConsumption;
        this.cyclesNumber    = cyclesNumber;
        this.earth           = earth;
    }

    /** Get the state.
     * @return state
     */
    public SpacecraftState getState() {
        return state;
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

    /** get the mass consumption.
     * @return mass consumption
     */
    public double getMassConsumption() {
        return massConsumption;
    }

    /** Get the number of cycles.
     * @return number of cycles
     */
    public int getCyclesNumber() {
        return cyclesNumber;
    }

    /** Get the Earth model.
     * @return Earth model
     */
    public BodyShape getEarth() {
        return earth;
    }

}
