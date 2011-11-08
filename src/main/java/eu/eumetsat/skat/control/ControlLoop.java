/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.analysis.MultivariateRealFunction;
import org.apache.commons.math.optimization.BaseMultivariateRealOptimizer;
import org.apache.commons.math.optimization.GoalType;
import org.apache.commons.math.optimization.RealPointValuePair;
import org.apache.commons.math.optimization.direct.CMAESOptimizer;
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
    private final TunableManeuver[] tunables;

    /** Parameters boundaries. */
    private final double[][] boundaries;

    /** Start point. */
    private final double[] startPoint;

    /** Station-keeping controls for single spacecraft. */
    private final List<MonitorableMonoSKControl> monoControls;

    /** Station-keeping controls for several spacecrafts. */
    private final List<MonitorableDuoSKControl> duoControls;

    /** Simple constructor.
     * <p>
     * Creates an empty control loop, with neither controls nor control parameters.
     * They must be added later on by {@link #addControl(double, SKControl)}
     * and {@link #addTunableManeuver(TunableManeuver)}.
     * </p>
     * @param spacecraftIndex index of the spacecraft controlled by this component
     * @param firstCycle first cycle this loop should control
     * @param lastCycle last cycle this loop should control
     * @param tunables tunable maneuvers (for all rolling cycles)
     * @param maxEval maximal number of objective function evaluations
     * @param optimizer optimizing engine
     * @param propagator orbit propagator
     * @param cycleDuration Cycle duration
     * @param rollingCycles number of cycles to use for rolling optimization
     */
    public ControlLoop(final int spacecraftIndex, final int firstCycle, final int lastCycle,
                       final TunableManeuver[] tunables, final int maxEval,
                       final BaseMultivariateRealOptimizer<MultivariateRealFunction> optimizer,
                       final Propagator propagator, final double cycleDuration, final int rollingCycles) {
        this.spacecraftIndex = spacecraftIndex;
        this.firstCycle      = firstCycle;
        this.lastCycle       = lastCycle;
        this.maxEval         = maxEval;
        this.optimizer       = optimizer;
        this.propagator      = propagator;
        this.tunables        = tunables.clone();
        this.monoControls    = new ArrayList<MonitorableMonoSKControl>();
        this.duoControls     = new ArrayList<MonitorableDuoSKControl>();
        this.cycleDuration   = cycleDuration;
        this.rollingCycles   = rollingCycles;

        // set the parameters boundaries and start point
        int nbParameters = 0;
        for (int i = 0; i < tunables.length; ++i) {
            for (final SKParameter parameter : tunables[i].getParameters()) {
                if (parameter.isTunable()) {
                    ++nbParameters;
                }
            }
        }

        this.boundaries = new double[2][nbParameters];
        this.startPoint = new double[nbParameters];

        int index = 0;
        for (int i = 0; i < tunables.length; ++i) {
            for (final SKParameter parameter : tunables[i].getParameters()) {
                if (parameter.isTunable()) {
                    boundaries[0][index] = parameter.getMin();
                    boundaries[1][index] = parameter.getMax();
                    startPoint[index]    = 0.5 * (parameter.getMin() + parameter.getMax());
                    ++index;
                }
            }
        }

    }

    /** Add a control law .
     * @param control control law to add
     */
    public void addControl(final MonitorableMonoSKControl control) {
        monoControls.add(control);
    }

    /** Add a control law .
     * @param control control law to add
     */
    public void addControl(final MonitorableDuoSKControl control) {
        duoControls.add(control);
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

            // set the reference consumed mass for maneuvers
            for (int i = 0; i < tunables.length; ++i) {
                final TunableManeuver tunable = tunables[i];
                SpacecraftState estimated = original.getEstimatedStartState();
                if (estimated == null) {
                    throw new SkatException(SkatMessages.NO_ESTIMATED_STATE,
                                            original.getName(), original.getCyclesNumber());
                }
                tunable.setReferenceConsumedMass(original.getBOLMass() -
                                                 original.getEstimatedStartState().getMass());
            }

            // find the optimal parameters that minimize objective function
            AbsoluteDate startDate  = original.getEstimatedStartState().getDate();
            AbsoluteDate targetDate = startDate.shiftedBy(cycleDuration * rollingCycles * Constants.JULIAN_DAY);
            System.out.println("starting optimization for cycle " + original.getCyclesNumber() +
                               "from " + startDate + " to " + targetDate);
            final List<SKControl> unmonitoredControls = new ArrayList<SKControl>(monoControls.size() + duoControls.size());
            for (final MonitorableMonoSKControl control : monoControls) {
                unmonitoredControls.add(control.getControlLaw());
            }
            for (final MonitorableDuoSKControl control : duoControls) {
                unmonitoredControls.add(control.getControlLaw());
            }
            final ObjectiveFunction objective =
                    new ObjectiveFunction(propagator, tunables, unmonitoredControls, targetDate,
                                          original.getEstimatedStartState(),
                                          original.getTheoreticalManeuvers());
            final RealPointValuePair pointValue =
                    optimizer.optimize(maxEval, objective, GoalType.MINIMIZE, startPoint, boundaries[0], boundaries[1]);
            final double[] optimum = pointValue.getPoint();
            for (int i = 0; i < optimum.length; ++i) {
                System.out.print((i == 0 ? "    " : ", ") + optimum[i]);
            }
            System.out.println(" -> " + pointValue.getValue());
            List<Double> fitness = ((CMAESOptimizer) optimizer).getStatisticsFitnessHistory();
            for (int i = 0; i < fitness.size(); ++i) {
                System.out.print((i == 0 ? "    fitness: " : ", ") + fitness.get(i));
            }
            System.out.println();
            List<Double> sigma = ((CMAESOptimizer) optimizer).getStatisticsSigmaHistory();
            for (int i = 0; i < fitness.size(); ++i) {
                System.out.print((i == 0 ? "    sigma: " : ", ") + sigma.get(i));
            }
            System.out.println();

            // perform a last run with monitoring enabled, using the optimum values
            final List<SKControl> monitoredControls = new ArrayList<SKControl>(monoControls.size() + duoControls.size());
            for (final MonitorableMonoSKControl control : monoControls) {
                control.setDate(startDate);
                monitoredControls.add(control);
            }
            for (final MonitorableDuoSKControl control : duoControls) {
                control.setDate(startDate);
                monitoredControls.add(control);
            }
            new ObjectiveFunction(propagator, tunables, monitoredControls, targetDate,
                                  original.getEstimatedStartState(),
                                  original.getTheoreticalManeuvers()).value(optimum);

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

            // prepare start point for next cycle by shifting already optimized maneuvers one cycle
            final int parametersPerCycle = tunables.length / rollingCycles;
            for (int i = 1; i < rollingCycles; ++i) {
                for (int j = 0; j < parametersPerCycle; ++j) {
                    startPoint[(i - 1) * rollingCycles + j] = startPoint[i * rollingCycles + j];
                }
            }

        }

        // return the updated states
        return updated;

    }

}
