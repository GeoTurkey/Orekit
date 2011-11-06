/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.analysis.MultivariateRealFunction;
import org.apache.commons.math.optimization.BaseMultivariateRealOptimizer;
import org.apache.commons.math.optimization.GoalType;
import org.apache.commons.math.optimization.RealPointValuePair;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;


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

    /** Index of the spacecraft controlled by this component. */
    private final int spacecraftIndex;

    /** Maximal number of objective function evaluations. */
    private final int maxEval;

    /** Optimizing engine. */
    private final BaseMultivariateRealOptimizer<MultivariateRealFunction> optimizer;

    /** Orbit propagator. */
    private final Propagator propagator;

    /** Cycle duration. */
    private final double cycleDuration;

    /** Number of cycles to use for rolling optimization. */
    private final int rollingCycles;

    /** First cycle this loop should control. */
    private final int firstCycle;

    /** Last cycle this loop should control. */
    private final int lastCycle;

    /** Tunable maneuvers. */
    private final List<TunableManeuver> tunables;

    /** Station-keeping controls. */
    private final List<SKControl> controls;

    /** Station-keeping parameters. */
    private final List<SKParameter> parameters;

    /** Simple constructor.
     * <p>
     * Creates an empty control loop, with neither controls nor control parameters.
     * They must be added later on by {@link #addControl(double, SKControl)}
     * and {@link #addTunableManeuver(TunableManeuver)}.
     * </p>
     * @param spacecraftIndex index of the spacecraft controlled by this component
     * @param firstCycle first cycle this loop should control
     * @param lastCycle last cycle this loop should control
     * @param maxEval maximal number of objective function evaluations
     * @param optimizer optimizing engine
     * @param propagator orbit propagator
     * @param cycleDuration Cycle duration
     * @param rollingCycles number of cycles to use for rolling optimization
     */
    public ControlLoop(final int spacecraftIndex, final int firstCycle, final int lastCycle,
                       final int maxEval,
                       final BaseMultivariateRealOptimizer<MultivariateRealFunction> optimizer,
                       final Propagator propagator,
                       final double cycleDuration, final int rollingCycles) {
        this.spacecraftIndex = spacecraftIndex;
        this.firstCycle      = firstCycle;
        this.lastCycle       = lastCycle;
        this.maxEval         = maxEval;
        this.optimizer       = optimizer;
        this.propagator      = propagator;
        this.tunables        = new ArrayList<TunableManeuver>();
        this.controls        = new ArrayList<SKControl>();
        this.parameters      = new ArrayList<SKParameter>();
        this.cycleDuration   = cycleDuration;
        this.rollingCycles   = rollingCycles;
    }

    /** Add a control law .
     * @param control control law to add
     */
    public void addControl(final SKControl control) {
        controls.add(control);
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

    /** {@inheritDoc} */
    public void setCycleEnd(final AbsoluteDate cycleEnd) {
        // nothing to do here (we handle several cycles at once, not only the current one)
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
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException, SkatException {

        ScenarioState[] updated = originals.clone();
        final ScenarioState original = originals[spacecraftIndex];

        if ((original.getCyclesNumber() >= firstCycle) && (original.getCyclesNumber() <= lastCycle)) {

            // guess a start point
            double[] lower = new double[parameters.size()];
            double[] upper = new double[parameters.size()];
            double[] startPoint = new double[parameters.size()];
            for (int j = 0; j < startPoint.length; ++j) {
                lower[j] = parameters.get(j).getMin();
                upper[j] = parameters.get(j).getMax();
                startPoint[j] = 0.5 * (lower[j] + upper[j]);
            }

            // set the reference date for maneuvers
            for (final TunableManeuver tunable : tunables) {
                SpacecraftState estimated = original.getEstimatedStartState();
                if (estimated == null) {
                    throw new SkatException(SkatMessages.NO_ESTIMATED_STATE,
                                            original.getName(), original.getCyclesNumber());
                }
                tunable.setReferenceDate(original.getEstimatedStartState().getDate());
                tunable.setReferenceConsumedMass(original.getBOLMass() -
                                                 original.getEstimatedStartState().getMass());
            }

            // find the optimal parameters that minimize objective function
            AbsoluteDate startDate  = original.getEstimatedStartState().getDate();
            AbsoluteDate targetDate = startDate.shiftedBy(rollingCycles * cycleDuration * Constants.JULIAN_DAY);
            System.out.println("starting optimization for cycle from " + startDate + " to " + targetDate);
            final ObjectiveFunction objective =
                    new ObjectiveFunction(propagator, parameters, controls, targetDate,
                                          original.getEstimatedStartState(),
                                          original.getTheoreticalManeuvers());
            final RealPointValuePair pointValue =
                    optimizer.optimize(maxEval, objective, GoalType.MINIMIZE, startPoint, lower, upper);
            final double[] optimum = pointValue.getPoint();
            System.out.print("cycle " + original.getCyclesNumber() + "[ " + startDate +
                             " ; " + targetDate + "]: ");
            for (int i = 0; i < optimum.length; ++i) {
                if (i > 0) {
                    System.out.print(", ");
                }
                System.out.print(optimum[i]);
            }
            System.out.println(" -> " + pointValue.getValue());

            // perform a last run so monitoring is updated with the optimal values
            objective.value(optimum);

            // update the scheduled maneuvers, adding the newly optimized set
            final List<ScheduledManeuver> theoreticalManeuvers = new ArrayList<ScheduledManeuver>();
            if (original.getTheoreticalManeuvers() != null) {
                theoreticalManeuvers.addAll(original.getTheoreticalManeuvers());
            }
            for (final TunableManeuver tunable : tunables) {
                // get the optimized maneuver, using the optimum value set above
                final ScheduledManeuver optimized = tunable.getManeuver();
                theoreticalManeuvers.add(optimized);
            }

            // build the updated scenario state
            updated[spacecraftIndex] = original.updateTheoreticalManeuvers(theoreticalManeuvers);
        }

        // return the updated states
        return updated;

    }

}
