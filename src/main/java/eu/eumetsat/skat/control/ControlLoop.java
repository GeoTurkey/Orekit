/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.analysis.MultivariateRealFunction;
import org.apache.commons.math.optimization.GoalType;
import org.apache.commons.math.optimization.MultivariateRealOptimizer;
import org.apache.commons.math.optimization.direct.BOBYQAOptimizer;
import org.apache.commons.math.optimization.direct.CMAESOptimizer;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;
import eu.eumetsat.skat.utils.SupportedOptimizer;


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

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Maximal number of objective function evaluations. */
    private final int maxEval;

    /** Number of sampling points. */
    private final int nbPoints;

    /** Optimizing engine. */
    private final SupportedOptimizer optimizer;

    /** Orbit propagators. */
    private final Propagator[] propagators;

    /** Cycle duration. */
    private final double cycleDuration;

    /** Number of cycles to use for rolling optimization. */
    private final int rollingCycles;

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
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param maxEval maximal number of objective function evaluations
     * @param nbPoints number of smapling points
     * @param optimizer optimizing engine
     * @param propagators orbit propagators
     * @param cycleDuration Cycle duration
     * @param rollingCycles number of cycles to use for rolling optimization
     */
    public ControlLoop(final int[] spacecraftIndices,
                       final int maxEval, final int nbPoints,
                       final SupportedOptimizer optimizer, final Propagator[] propagators,
                       final double cycleDuration, final int rollingCycles) {
        this.spacecraftIndices = spacecraftIndices.clone();
        this.maxEval           = maxEval;
        this.nbPoints          = nbPoints;
        this.optimizer         = optimizer;
        this.propagators       = propagators.clone();
        tunables               = new ArrayList<TunableManeuver>();
        controls               = new ArrayList<ScaledControl>();
        parameters             = new ArrayList<SKParameter>();
        this.cycleDuration     = cycleDuration;
        this.rollingCycles     = rollingCycles;
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
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException, SkatException {

        ScenarioState[] updated = originals.clone();

        for (int i = 0; i < spacecraftIndices.length; ++i) {

            // select the current spacecraft affected by this component
            final int index = spacecraftIndices[i];

            // guess a start point
            double[] startPoint = new double[parameters.size()];
            for (int j = 0; j < startPoint.length; ++j) {
                startPoint[j] = parameters.get(j).guessOptimalValue();
            }

            // set the reference date for maneuvers
            for (final TunableManeuver tunable : tunables) {
                SpacecraftState estimated = originals[index].getEstimatedStartState();
                if (estimated == null) {
                    throw new SkatException(SkatMessages.NO_ESTIMATED_STATE, originals[index].getName());
                }
                tunable.setReferenceDate(originals[index].getEstimatedStartState().getDate());
            }

            // find the optimal parameters that minimize objective function
            // TODO introduce constraints
            AbsoluteDate startDate  = originals[index].getEstimatedStartState().getDate();
            AbsoluteDate targetDate = startDate.shiftedBy(rollingCycles * cycleDuration);
            final MultivariateRealFunction objective =
                    new ObjectiveFunction(propagators[i], parameters, controls, targetDate,
                                          originals[index].getEstimatedStartState(),
                                          originals[index].getTheoreticalManeuvers());
            MultivariateRealOptimizer o;
            switch (optimizer) {
            case CMA_ES :
                o = new CMAESOptimizer(nbPoints);
                break;
            case BOBYQA :
                o = new BOBYQAOptimizer(nbPoints);
                break;
            default :
                throw SkatException.createInternalError(null);
            }
            double[] optimum = o.optimize(maxEval, objective, GoalType.MINIMIZE, startPoint).getPoint();

            // set the parameters to the optimal values
            for (int j = 0; j < optimum.length; ++j) {
                final SKParameter parameter = parameters.get(j);
                parameter.storeLastOptimalValue(optimum[j]);
                parameter.setValue(optimum[j]);
            }

            // update the scheduled maneuvers, adding the newly optimized set
            final List<ScheduledManeuver> theoreticalManeuvers = new ArrayList<ScheduledManeuver>();
            theoreticalManeuvers.addAll(originals[index].getTheoreticalManeuvers());
            for (final TunableManeuver tunable : tunables) {
                // get the optimized maneuver, using the optimum value set above
                final ScheduledManeuver optimized = tunable.getManeuver();
                theoreticalManeuvers.add(optimized);
            }

            // build the updated scenario state
            updated[index] = originals[index].updateTheoreticalManeuvers(theoreticalManeuvers);

        }

        // return the updated states
        return updated;

    }

}
