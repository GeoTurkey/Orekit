/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.analysis.MultivariateFunction;
import org.apache.commons.math.optimization.BaseMultivariateRealOptimizer;
import org.apache.commons.math.optimization.GoalType;
import org.apache.commons.math.optimization.RealPointValuePair;
import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.ConstantThrustManeuver;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.ManeuverAdapterPropagator;
import org.orekit.propagation.events.DateDetector;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;
import eu.eumetsat.skat.utils.SupportedPropagator.PropagatorRandomizer;


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
    private final BaseMultivariateRealOptimizer<MultivariateFunction> optimizer;

    /** Orbit propagator randomizer. */
    private final PropagatorRandomizer randomizer;

    /** Cycle duration. */
    private final double cycleDuration;

    /** Number of cycles to use for rolling optimization. */
    private final int rollingCycles;

    /** Threshold for eliminating to small in-plane maneuvers. */
    private final double inPlaneEliminationThreshold;

    /** Threshold for eliminating to small out_of-plane maneuvers. */
    private final double outOfPlaneEliminationThreshold;

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

    /** Station-keeping control laws. */
    private final List<SKControl> controls;

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
     * @param randomizer orbit propagator randomizer
     * @param cycleDuration Cycle duration
     * @param rollingCycles number of cycles to use for rolling optimization
     * @param inPlaneEliminationThreshold threshold for eliminating to small in-plane maneuvers
     * @param outOfPlaneEliminationThreshold threshold for eliminating to small out_of-plane maneuvers
     */
    public ControlLoop(final int spacecraftIndex, final int firstCycle, final int lastCycle,
                       final TunableManeuver[] tunables, final int maxEval,
                       final BaseMultivariateRealOptimizer<MultivariateFunction> optimizer,
                       final PropagatorRandomizer randomizer, final double cycleDuration, final int rollingCycles,
                       final double inPlaneEliminationThreshold, final double outOfPlaneEliminationThreshold) {
        this.spacecraftIndex                = spacecraftIndex;
        this.firstCycle                     = firstCycle;
        this.lastCycle                      = lastCycle;
        this.maxEval                        = maxEval;
        this.optimizer                      = optimizer;
        this.randomizer                     = randomizer;
        this.tunables                       = tunables.clone();
        this.controls                       = new ArrayList<SKControl>();
        this.cycleDuration                  = cycleDuration;
        this.rollingCycles                  = rollingCycles;
        this.inPlaneEliminationThreshold    = inPlaneEliminationThreshold;
        this.outOfPlaneEliminationThreshold = outOfPlaneEliminationThreshold;

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
     * @param controlLaw control law to add
     */
    public void addControl(final SKControl controlLaw) {
        controls.add(controlLaw);
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
                SpacecraftState estimated = original.getEstimatedState();
                if (estimated == null) {
                    throw new SkatException(SkatMessages.NO_ESTIMATED_STATE,
                                            original.getName(), original.getCyclesNumber());
                }
                tunable.setReferenceConsumedMass(original.getBOLMass() -
                                                 original.getEstimatedState().getMass());
            }

            // compute a reference ephemeris, on which tunable maneuvers will be added
            final BoundedPropagator reference =
                    computeReferenceEphemeris(original.getEstimatedState(), original.getManeuvers());

            // find the optimal parameters that minimize objective function
            final ObjectiveFunction objective =
                    new ObjectiveFunction(reference, reference.getMinDate(), reference.getMaxDate(),
                                          cycleDuration, rollingCycles, tunables, controls);
            final RealPointValuePair pointValue =
                    optimizer.optimize(maxEval, objective, GoalType.MINIMIZE, startPoint);
            final double[] optimum = pointValue.getPoint();

            // update the scheduled maneuvers, adding the newly optimized set
            final List<ScheduledManeuver> theoreticalManeuvers = new ArrayList<ScheduledManeuver>();
            if (original.getManeuvers() != null) {
                theoreticalManeuvers.addAll(original.getManeuvers());
            }
            final ScheduledManeuver[] maneuvers = objective.setUpManeuvers(optimum, new ManeuverAdapterPropagator(reference));
            for (int i = 0; i < tunables.length / rollingCycles; ++i) {
                // extract the optimized maneuver for the next cycle only
                if (maneuvers[i].isInPlane()) {
                    if (maneuvers[i].getDeltaV().getNorm() >= inPlaneEliminationThreshold) {
                        theoreticalManeuvers.add(maneuvers[i]);
                    }
                } else {
                    if (maneuvers[i].getDeltaV().getNorm() >= outOfPlaneEliminationThreshold) {
                        theoreticalManeuvers.add(maneuvers[i]);
                    }
                }
            }

            // build the updated scenario state
            updated[spacecraftIndex] = original.updateManeuvers(theoreticalManeuvers);

            // prepare start point for next cycle by shifting already optimized maneuvers one cycle
            // and repeating last cycle
            if (rollingCycles > 1) {
                final int parametersPerCycle = startPoint.length / rollingCycles;
                System.arraycopy(optimum,    parametersPerCycle,
                                 startPoint, 0,
                                 startPoint.length - parametersPerCycle);
                System.arraycopy(startPoint, startPoint.length - 2 * parametersPerCycle,
                                 startPoint, startPoint.length - parametersPerCycle,
                                 parametersPerCycle);
            }

        }

        // return the updated states
        return updated;

    }

    /** Set up reference ephemeris, without the tunable maneuvers.
     * @param initialState initial state
     * @param scheduledManeuvers maneuvers that are already scheduled
     * and hence not optimized themselves, may be null
     * @exception OrekitException if propagation cannot be performed
     */
    private BoundedPropagator computeReferenceEphemeris(final SpacecraftState initialState,
                                                        final List<ScheduledManeuver> scheduledManeuvers)
        throws OrekitException {

        // set up the propagator with the station-keeping elements
        // that are part of the optimization process in the control loop
        final Propagator propagator = randomizer.getPropagator(initialState);
        propagator.setEphemerisMode();

        if (scheduledManeuvers != null) {
            // set up the scheduled maneuvers that are not optimized
            for (final ScheduledManeuver maneuver : scheduledManeuvers) {
                if (propagator instanceof NumericalPropagator) {
                    final Propagator p = maneuver.getTrajectory();
                    final double duration = maneuver.getDuration(p.propagate(maneuver.getDate()).getMass());
                    final AbsoluteDate startMan = maneuver.getDate().shiftedBy(-0.5 * duration);
                    final ConstantThrustManeuver ctm =
                            new ConstantThrustManeuver(startMan, duration,
                                                       maneuver.getThrust(), maneuver.getIsp(),
                                                       maneuver.getDeltaV().normalize());
                    ((NumericalPropagator) propagator).addForceModel(ctm);
                } else {
                    propagator.addEventDetector(new ImpulseManeuver(new DateDetector(maneuver.getDate()),
                                                                    maneuver.getDeltaV(),
                                                                    maneuver.getIsp()));
                }
            }
        }

        // perform propagation
        final double propagationDuration = rollingCycles * cycleDuration * Constants.JULIAN_DAY;
        final AbsoluteDate endDate = initialState.getDate().shiftedBy(propagationDuration);
        propagator.propagate(endDate);

        return propagator.getGeneratedEphemeris();

    }

}
