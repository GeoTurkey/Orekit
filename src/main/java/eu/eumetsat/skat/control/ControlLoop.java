/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.apache.commons.math.analysis.MultivariateRealFunction;
import org.apache.commons.math.optimization.GoalType;
import org.apache.commons.math.optimization.MultivariateRealOptimizer;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;


/**
 * Control loop between {@link SKParametersList control parameters} and {@link
 * SKControl station-keeping controls}.
 * <p>
 * This loop is mainly an optimization loop that adjusts the control parameters
 * in order to minimize an objective function J defined as:<br>
 *   J = &sum;(((control<sub>i</sub>.{@link SKControl#getAchievedValue() getAchievedValue()}
 *             - control<sub>i</sub>.{@link SKControl#getTargetValue() getTargetValue()})
 *             / scale<sub>i</sub>)<sup>2</sup>)<br>
 * the sum being computed across all scaled controls.
 * </p>
 * @see SKParametersList
 * @see SKControl
 * @author Luc Maisonobe
 */
public class ControlLoop implements ScenarioComponent {

    /** Maximal number of objective function evaluations. */
    private final int maxEval;

    /** Optimizing engine. */
    private final MultivariateRealOptimizer optimizer;

    /** Orbit propagator. */
    private final Propagator propagator;

    /** Station-keeping controls. */
    private final List<ScaledControl> controls;

    /** Tunable control parameters. */
    private final List<SKParameter> parameters;

    /** Simple constructor.
     * <p>
     * Creates an empty control loop, with neither controls nor control parameters.
     * They must be added later on by {@link #addControl(double, SKControl)}
     * and {@link #addParametersList(SKParametersList)}.
     * </p>
     * @param maxEval maximal number of objective function evaluations
     * @param optimizer optimizing engine
     * @param propagator orbit propagator
     */
    public ControlLoop(final int maxEval, final MultivariateRealOptimizer optimizer,
                       final Propagator propagator) {
        this.maxEval    = maxEval;
        this.optimizer  = optimizer;
        this.propagator = propagator;
        controls        = new ArrayList<ScaledControl>();
        parameters      = new ArrayList<SKParameter>();
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
     * @param parametersList control parameters list to use
     */
    public void addParametersList(final SKParametersList parametersList) {
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
     * #addParametersList(SKParametersList) control parameters} values
     * will be set to the optimal values that best fulfill the {@link
     * #addControl(double, SKControl) station keeping controls}.
     * </p>
     */
    public ScenarioState updateState(final ScenarioState origin, final AbsoluteDate target)
        throws OrekitException {

        // compute start point
        double[] startPoint = new double[parameters.size()];
        for (int i = 0; i < startPoint.length; ++i) {
            startPoint[i] = parameters.get(i).getValue();
        }

        // find the optimal parameters that minimize objective function
        double[] optimum = optimizer.optimize(maxEval,
                                              new ObjectiveFunction(), GoalType.MINIMIZE,
                                              startPoint).getPoint();

        // set the parameters to the optimal values
        for (int i = 0; i < optimum.length; ++i) {
            parameters.get(i).setValue(optimum[i]);
        }

        // TODO what else ?
        
        return null;

    }

    /** Local function to be optimized. */
    private class ObjectiveFunction implements MultivariateRealFunction {

        /** {@inheritDoc} */
        public double value(final double[] point) {

            // set the parameters to the current test values
            for (int i = 0; i < point.length; ++i) {
                parameters.get(i).setValue(point[i]);
            }

            // get the detectors associated with the station-keeping elements
            // (note that some elements may be associated with the same detector)
            Set<EventDetector> detectors = new HashSet<EventDetector>();
            for (final SKParameter parameter : parameters) {
                if (parameter.getEventDetector() != null) {
                    detectors.add(parameter.getEventDetector());
                }
            }
            for (final ScaledControl sc : controls) {
                if (sc.getControl().getEventDetector() != null) {
                    detectors.add(sc.getControl().getEventDetector());
                }
            }

            // get the step handlers associated with the station-keeping elements
            // (note that some elements may be associated with the same step handler)
            Set<OrekitStepHandler> handlers = new HashSet<OrekitStepHandler>();
            for (final SKParameter parameter : parameters) {
                if (parameter.getStepHandler() != null) {
                    handlers.add(parameter.getStepHandler());
                }
            }
            for (final ScaledControl sc : controls) {
                if (sc.getControl().getStepHandler() != null) {
                    handlers.add(sc.getControl().getStepHandler());
                }
            }

            // set up the propagator
            propagator.clearEventsDetectors();
            for (final EventDetector detector : detectors) {
                propagator.addEventDetector(detector);
            }
            if (handlers.size() == 1) {
                propagator.setMasterMode(handlers.iterator().next());
            } else if (handlers.size() > 1) {
                // TODO check what to do when 0
                throw OrekitException.createInternalError(null);
            }

            // perform propagation
            propagator.resetInitialState(...);
            propagator.propagate(...);

            // compute sum of squared scaled residuals
            double sum = 0;
            for (final ScaledControl s : controls) {
                sum += s.getScaledResidualSquared();
            }

            return sum;

        }

    }

}
