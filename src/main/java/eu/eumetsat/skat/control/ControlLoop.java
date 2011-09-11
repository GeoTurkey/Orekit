/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;

import org.orekit.errors.OrekitException;
import org.orekit.propagation.SpacecraftState;

import eu.eumetsat.skat.CycleComponent;


/**
 * Class representing a control loop between {@link ControlParameter control
 * parameters} and {@link StationKeepingGoal station-keeping goals}.
 * <p>
 * This loop is mainly an optimization loop that adjust the control parameters
 * in order to minimize an objective function J defined as:<br>
 *   J = &sum;(((goal<sub>i</sub>.{@link StationKeepingGoal#getAchievedValue() getAchievedValue()}
 *             - goal<sub>i</sub>.{@link StationKeepingGoal#getTarget() getTarget()})
 *             / scale<sub>i</sub>)<sup>2</sup>)<br>
 * the sum being computed across all scaled goals.
 * </p>
 * @see ControlParameter
 * @see StationKeepingGoal
 * @author Luc Maisonobe
 */
public class ControlLoop implements CycleComponent {

    /** Station-keeping goals. */
    private List<ScaledGoal> goals;

    /** Control parameters. */
    private List<ControlParameter> parameters;

    /** Simple constructor.
     * <p>
     * Creates an empty control loop, with neither goals nor control parameters.
     * They must be added later on by {@link #addGoal(double, StationKeepingGoal)}
     * and {@link #addControlParameter(ControlParameter)}.
     * </p>
     */
    public ControlLoop() {
        goals      = new ArrayList<ScaledGoal>();
        parameters = new ArrayList<ControlParameter>();
    }

    /** Add a scaled goal.
     * <p>
     * The scale of the goal is a scalar parameter with the same physical unit
     * as the goal itself (radians, meters, seconds ...). It's purpose is to
     * allow mixing goals in a global scalar objective function by computing<br>
     *   J = &sum;(((goal<sub>i</sub>.{@link StationKeepingGoal#getAchievedValue() getAchievedValue()}
     *             - goal<sub>i</sub>.{@link StationKeepingGoal#getTarget() getTarget()})
     *             / scale<sub>i</sub>)<sup>2</sup>)<br>
     * the sum being computed across all scaled goals.
     * </p>
     * @param scale scale of the goal
     * @param goal goal to add
     */
    public void addGoal(final double scale, final StationKeepingGoal goal) {
        goals.add(new ScaledGoal(scale, goal));
    }

    /** Add a control parameter.
     * @param parameter control parameter to add
     */
    public void addControlParameter(final ControlParameter parameter) {
        parameters.add(parameter);
    }

    /** Optimize the control parameters to achieve the goals.
     * <p>
     * At the end of the optimization the {@link
     * #addControlParameter(ControlParameter) control parameters} values
     * will be set to the optimal values that best fulfill the {@link
     * #addGoal(StationKeepingGoal) station keeping goals}.
     * </p>
     * @param initialState spacecraft state at cycle start
     * @param duration cycle duration to simulate
     * @return theoretical state at cycle end
     * @exception OrekitException if simulation cannot be performed
     */
    public SpacecraftState run(final SpacecraftState initialState,
                               final double duration)
        throws OrekitException {
        // TODO
        return null;
    }

}
