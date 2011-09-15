/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;

import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;


/**
 * Class representing a control loop between {@link SKParametersList control
 * parameters} and {@link SKControl station-keeping controls}.
 * <p>
 * This loop is mainly an optimization loop that adjust the control parameters
 * in order to minimize an objective function J defined as:<br>
 *   J = &sum;(((control<sub>i</sub>.{@link SKControl#getAchievedValue() getAchievedValue()}
 *             - control<sub>i</sub>.{@link SKControl#getTarget() getTarget()})
 *             / scale<sub>i</sub>)<sup>2</sup>)<br>
 * the sum being computed across all scaled controls.
 * </p>
 * @see SKParametersList
 * @see SKControl
 * @author Luc Maisonobe
 */
public class ControlLoop implements ScenarioComponent {

    /** Station-keeping controls. */
    private List<ScaledControl> controls;

    /** Tunable control parameters. */
    private List<SKParameter> parameters;

    /** Simple constructor.
     * <p>
     * Creates an empty control loop, with neither controls nor control parameters.
     * They must be added later on by {@link #addControl(double, SKControl)}
     * and {@link #addControlParametersList(SKParametersList)}.
     * </p>
     */
    public ControlLoop() {
        controls      = new ArrayList<ScaledControl>();
        parameters = new ArrayList<SKParameter>();
    }

    /** Add a scaled control.
     * <p>
     * The scale of the control is a scalar parameter with the same physical unit
     * as the control itself (radians, meters, seconds ...). It's purpose is to
     * allow mixing controls in a global scalar objective function by computing<br>
     *   J = &sum;(((control<sub>i</sub>.{@link SKControl#getAchievedValue() getAchievedValue()}
     *             - control<sub>i</sub>.{@link SKControl#getTarget() getTarget()})
     *             / scale<sub>i</sub>)<sup>2</sup>)<br>
     * the sum being computed across all scaled controls.
     * </p>
     * @param scale scale of the control
     * @param control control to add
     */
    public void addControl(final double scale, final SKControl control) {
        controls.add(new ScaledControl(scale, control));
    }

    /** Add the tunable parameters from a control parameters list.
     * @param parametersList control parameters list to use
     */
    public void addControlParametersList(final SKParametersList parametersList) {
        for (final SKParameter parameter : parametersList.getParameters()) {
            if (parameter.isTunable()) {
                parameters.add(parameter);
            }
        }
    }

    /** {@inheritDoc}
     * <p>
     * Optimize the control parameters to achieve the controls.
     * </p>
     * <p>
     * At the end of the optimization the {@link
     * #addControlParametersList(SKParametersList) control parameters} values
     * will be set to the optimal values that best fulfill the {@link
     * #addControl(double, SKControl) station keeping controls}.
     * </p>
     */
    public ScenarioState updateState(final ScenarioState origin, final AbsoluteDate target)
        throws OrekitException {
        // TODO
        return null;
    }

}
