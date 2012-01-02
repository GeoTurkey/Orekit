/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.ConstantThrustManeuver;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.ManeuverAdapterPropagator;
import org.orekit.propagation.events.DateDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.propagation.sampling.OrekitStepHandlerMultiplexer;
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

    /** Maximal number of iterations. */
    private final int maxIter;

    /** Orbit propagator randomizer. */
    private final PropagatorRandomizer randomizer;

    /** Cycle duration. */
    private final double cycleDuration;

    /** First cycle this loop should control. */
    private final int firstCycle;

    /** Last cycle this loop should control. */
    private final int lastCycle;

    /** Tunable maneuvers. */
    private final TunableManeuver[] tunables;

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
     * @param maxIter maximal number of iterations
     * @param randomizer orbit propagator randomizer
     * @param cycleDuration Cycle duration
     */
    public ControlLoop(final int spacecraftIndex, final int firstCycle, final int lastCycle,
                       final TunableManeuver[] tunables, final int maxIter,
                       final PropagatorRandomizer randomizer,
                       final double cycleDuration) {
        this.spacecraftIndex = spacecraftIndex;
        this.firstCycle      = firstCycle;
        this.lastCycle       = lastCycle;
        this.maxIter         = maxIter;
        this.randomizer      = randomizer;
        this.tunables        = tunables.clone();
        this.controls        = new ArrayList<SKControl>();
        this.cycleDuration   = cycleDuration;
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
            ScheduledManeuver[] maneuvers = new ScheduledManeuver[0];

            // find the optimal parameters that fulfill control laws
            boolean converged = false;
            for (int iter = 0; iter < maxIter && !converged; ++iter) {

                // compute the control laws
                final ManeuverAdapterPropagator propagator;
                if (maneuvers.length == 0) {
                    propagator = new ManeuverAdapterPropagator(reference);
                } else {
                    propagator = maneuvers[0].getTrajectory();
                }
                runCycle(iter, maneuvers, propagator, original.getManeuvers(),
                         reference.getMinDate(), reference.getMaxDate());

                // prepare maneuver update
                ScheduledManeuver[] tuned = maneuvers.clone();

                // update the maneuvers
                for (final SKControl controlLaw : controls) {
                    tuned = controlLaw.tuneManeuvers(tuned, reference);
                }

                // check convergence
                converged = true;
                if (tuned.length != maneuvers.length) {
                    converged = false;
                } else {
                    for (int i = 0; i < tuned.length; ++i) {
                        converged &= tuned[i].isWithinThreshold(maneuvers[i]);
                    }
                }

                maneuvers = tuned;

            }

            if (!converged) {
                final StringBuilder builder = new StringBuilder();
                for (int j = 0; j < controls.size(); ++j) {
                    if (j > 0) {
                        builder.append(", ");
                    }
                    builder.append(controls.get(j).getName());
                }
                throw new SkatException(SkatMessages.NO_CONVERGENCE, maxIter, builder.toString());
            }

            // update the scheduled maneuvers, adding the newly optimized set
            final List<ScheduledManeuver> theoreticalManeuvers =
                    new ArrayList<ScheduledManeuver>(original.getManeuvers());
            for (final ScheduledManeuver maneuver : maneuvers) {
                // extract the optimized maneuver for the next cycle only
                if (maneuver.getDeltaV().getNorm() >= maneuver.getModel().getEliminationThreshold()) {
                    theoreticalManeuvers.add(maneuver);
                }
            }

            // build the updated scenario state
            updated[spacecraftIndex] = original.updateManeuvers(theoreticalManeuvers);

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
                    final double duration = maneuver.getDuration(maneuver.getStateBefore().getMass());
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
        final double propagationDuration = cycleDuration * Constants.JULIAN_DAY;
        final AbsoluteDate endDate = initialState.getDate().shiftedBy(propagationDuration);
        propagator.propagate(endDate);

        return propagator.getGeneratedEphemeris();

    }

    /** Run one iteration of the cycle.
     * @param iteration iteration number
     * @param maneuvers current value of the tuned maneuvers
     * @param propagator propagator to use (already takes the maneuvers into account)
     * @param fixedManeuvers list of maneuvers already fixed for the cycle
     * @param start start of the simulation
     * @param end end of the simulation
     * @exception OrekitException if the propagation cannot be performed
     * @exception SkatException if run cannot be initialized
     */
    public void runCycle(final int iteration, final ScheduledManeuver[] maneuvers,
                         final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                         final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException, SkatException {

        // prepare run
        for (final SKControl control : controls) {
            control.initializeRun(iteration, maneuvers, propagator, fixedManeuvers, start, end);
        }

        // get the detectors associated with control laws
        final Set<EventDetector> detectors = new HashSet<EventDetector>();
        for (final SKControl control : controls) {
            if (control.getEventDetector() != null) {
                detectors.add(control.getEventDetector());
            }
        }

        // get the step handlers associated with control laws
        final OrekitStepHandlerMultiplexer multiplexer = new OrekitStepHandlerMultiplexer();
        for (final SKControl control : controls) {
            if (control.getStepHandler() != null) {
                multiplexer.add(control.getStepHandler());
            }
        }
        propagator.setMasterMode(multiplexer);

        // set up the propagator with the station-keeping elements
        // that are part of the optimization process in the control loop
        for (final EventDetector detector : detectors) {
            propagator.addEventDetector(detector);
        }

        // perform propagation
        propagator.propagate(start.shiftedBy(cycleDuration * Constants.JULIAN_DAY));

    }

}
