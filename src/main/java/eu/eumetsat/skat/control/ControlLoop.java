/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.analysis.MultivariateRealFunction;
import org.apache.commons.math.optimization.GoalType;
import org.apache.commons.math.optimization.MultivariateRealOptimizer;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.Propagator;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;


/**
 * Control loop between {@link SKParameter control parameters} and {@link
 * SKControl station-keeping controls}.
 * <p>
 * This loop is mainly an optimization loop that adjusts the control parameters
 * in order to minimize an objective function J defined as:<br>
 *   J = &sum;(((control<sub>i</sub>.{@link SKControl#getAchievedValue() getAchievedValue()}
 *             - control<sub>i</sub>.{@link SKControl#getTargetValue() getTargetValue()})
 *             / scale<sub>i</sub>)<sup>2</sup>)<br>
 * the sum being computed across all scaled controls.
 * </p>
 * @see TunableManeuver
 * @see SKParameter
 * @see SKControl
 * @author Luc Maisonobe
 */
public class ControlLoop implements ScenarioComponent {

    /** Index of the spacecraft managed by this loop. */
    private final int spacecraftIndex;

    /** Maximal number of objective function evaluations. */
    private final int maxEval;

    /** Optimizing engine. */
    private final MultivariateRealOptimizer optimizer;

    /** Orbit propagator. */
    private final Propagator propagator;

    /** Tunable maneuvers. */
    private final List<TunableManeuver> tunables;

    /** Station-keeping controls. */
    private final List<ScaledControl> controls;

    /** Station-keeping parameters. */
    private final List<SKParameter> parameters;

    /** Simple constructor.
     * <p>
     * Creates an empty control loop, with neither controls nor control parameters.
     * They must be added later on by {@link #addControl(double, SKControl)}
     * and {@link #addTunableManeuver(TunableManeuver)}.
     * </p>
     * @param spacecraftIndex index of the spacecraft managed by this loop
     * @param maxEval maximal number of objective function evaluations
     * @param optimizer optimizing engine
     * @param propagator orbit propagator
     */
    public ControlLoop(final int spacecraftIndex,
                       final int maxEval, final MultivariateRealOptimizer optimizer,
                       final Propagator propagator) {
        this.spacecraftIndex = spacecraftIndex;
        this.maxEval         = maxEval;
        this.optimizer       = optimizer;
        this.propagator      = propagator;
        tunables             = new ArrayList<TunableManeuver>();
        controls             = new ArrayList<ScaledControl>();
        parameters           = new ArrayList<SKParameter>();
    }

    /** Add a scaled control.
     * <p>
     * The scale of the control is a scalar parameter with the same physical unit
     * as the control itself (radians, meters, seconds ...). It's purpose is to
     * allow mixing controls in a global scalar objective function by computing<br>
     *   J = &sum;(((control<sub>i</sub>.{@link SKControl#getAchievedValue() getAchievedValue()}
     *             - control<sub>i</sub>.{@link SKControl#getTargetValue() getTargetValue()})
     *             / scale<sub>i</sub>)<sup>2</sup>)<br>
     * the sum being computed across all scaled controls.
     * </p>
     * @param scale scale of the control (must have the same physical unit as the control value)
     * @param control control to add
     */
    public void addControl(final double scale, final SKControl control) {
        controls.add(new ScaledControl(scale, control));
    }

    /** Add the tunable parameters from a control parameters list.
     * @param tunable maneuver that should be tuned by this control loop
     */
    public void addTunableManeuver(final TunableManeuver tunable) {

        // store a reference to the maneuver
        tunables.add(tunable);

        // extract the station-keeping parameters
        for (final SKParameter parameter : tunable.getParameters()) {
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
     * #addTunableManeuver(TunableManeuver) maneuvers} values
     * will be set to the optimal values that best fulfill the {@link
     * #addControl(double, SKControl) station keeping controls}.
     * </p>
     */
    public ScenarioState[] updateStates(final ScenarioState[] originals, final AbsoluteDate target)
        throws OrekitException {

        // guess a start point
        double[] startPoint = new double[parameters.size()];
        for (int i = 0; i < startPoint.length; ++i) {
            startPoint[i] = parameters.get(i).guessOptimalValue();
        }

        // set the reference date for maneuvers
        for (final TunableManeuver tunable : tunables) {
            tunable.setReferenceDate(originals[spacecraftIndex].getEstimatedStartState().getDate());
        }

        // find the optimal parameters that minimize objective function
        // TODO introduce constraints
        final MultivariateRealFunction objective =
                new ObjectiveFunction(propagator, parameters, controls, target,
                                      originals[spacecraftIndex].getEstimatedStartState(),
                                      originals[spacecraftIndex].getTheoreticalManeuvers());
        double[] optimum =
                optimizer.optimize(maxEval, objective, GoalType.MINIMIZE, startPoint).getPoint();

        // set the parameters to the optimal values
        for (int i = 0; i < optimum.length; ++i) {
            final SKParameter parameter = parameters.get(i);
            parameter.storeLastOptimalValue(optimum[i]);
            parameter.setValue(optimum[i]);
        }

        // update the scheduled maneuvers, adding the newly optimized set
        final List<ScheduledManeuver> theoreticalManeuvers = new ArrayList<ScheduledManeuver>();
        theoreticalManeuvers.addAll(originals[spacecraftIndex].getTheoreticalManeuvers());
        for (final TunableManeuver tunable : tunables) {
            // get the optimized maneuver, using the optimum value set above
            final ScheduledManeuver optimized = tunable.getManeuver();
            theoreticalManeuvers.add(optimized);
        }

        // build the updated scenario state
        ScenarioState[] updated = originals.clone();
        updated[spacecraftIndex] = originals[spacecraftIndex].updateTheoreticalManeuvers(theoreticalManeuvers);
        return updated;

    }

}
