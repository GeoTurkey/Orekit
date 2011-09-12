/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;

import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;


/**
 * Class representing a control loop between {@link ControlParametersSet control
 * parameters} and {@link StationKeepingGoal station-keeping goals}.
 * <p>
 * This loop is mainly an optimization loop that adjust the control parameters
 * in order to minimize an objective function J defined as:<br>
 *   J = &sum;(((goal<sub>i</sub>.{@link StationKeepingGoal#getAchievedValue() getAchievedValue()}
 *             - goal<sub>i</sub>.{@link StationKeepingGoal#getTarget() getTarget()})
 *             / scale<sub>i</sub>)<sup>2</sup>)<br>
 * the sum being computed across all scaled goals.
 * </p>
 * @see ControlParametersSet
 * @see StationKeepingGoal
 * @author Luc Maisonobe
 */
public class ControlLoop implements ScenarioComponent {

    /** Station-keeping goals. */
    private List<ScaledGoal> goals;

    /** Tunable control parameters. */
    private List<ControlParameter> parameters;

    /** Simple constructor.
     * <p>
     * Creates an empty control loop, with neither goals nor control parameters.
     * They must be added later on by {@link #addGoal(double, StationKeepingGoal)}
     * and {@link #addControlParametersSet(ControlParametersSet)}.
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

    /** Add the tunable parameters from a control parameters set.
     * @param parametersSet control parameters set to use
     */
    public void addControlParametersSet(final ControlParametersSet parametersSet) {
        for (final ControlParameter parameter : parametersSet.getParameters()) {
            if (parameter.isTunable()) {
                parameters.add(parameter);
            }
        }
    }

    /** {@inheritDoc}
     * <p>
     * Optimize the control parameters to achieve the goals.
     * </p>
     * <p>
     * At the end of the optimization the {@link
     * #addControlParametersSet(ControlParametersSet) control parameters} values
     * will be set to the optimal values that best fulfill the {@link
     * #addGoal(StationKeepingGoal) station keeping goals}.
     * </p>
     */
    public ScenarioState apply(final ScenarioState origin, final AbsoluteDate target)
        throws OrekitException {
        // TODO
        return null;
    }

}
