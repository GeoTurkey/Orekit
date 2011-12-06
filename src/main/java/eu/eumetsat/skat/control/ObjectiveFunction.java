/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.apache.commons.math.analysis.MultivariateFunction;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.ManeuverAdapterPropagator;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandlerMultiplexer;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;

/** Objective function to be optimized by the {@link ControlLoop}.
 * @see ControlLoop
 * @author Luc Maisonobe
 */
class ObjectiveFunction implements MultivariateFunction {

    /** Reference ephemeris. */
    private final BoundedPropagator reference;

    /** Station-keeping controls. */
    private final List<SKControl> controls;

    /** Tunable maneuvers. */
    private final TunableManeuver[] tunables;

    /** Number of cycles to use for rolling optimization. */
    private final int rollingCycles;

    /** Duration of cycle. */
    private final double cycleDuration;

    /** Initial state. */
    private final SpacecraftState initialState;

    /** Simple constructor.
     * @param reference reference ephemeris on which maneuvers will be added
     * @param tunables station-keeping maneuvers
     * @param cycleDuration Cycle duration
     * @param rollingCycles number of cycles to use for rolling optimization
     * @param controls station-keeping controls
     * @param initialState initial state
     */
    public ObjectiveFunction(final BoundedPropagator reference, final TunableManeuver[] tunables,
                             final double cycleDuration, final int rollingCycles,
                             final List<SKControl> controls, final SpacecraftState initialState) {

        this.reference     = reference;
        this.tunables      = tunables.clone();
        this.rollingCycles = rollingCycles;
        this.cycleDuration = cycleDuration;
        this.controls      = controls;
        this.initialState  = initialState;

    }

    /** Set up the station keeping maneuvers parameters corresponding to a set of values.
     * @param point values to use for the station keeping maneuvers parameters
     * @param propagator maneuver adapter to use (the maneuvers will be added to it)
     * @return maneuvers that were added to the propagator
     * @see #value(double[])
     * @exception OrekitException if spacecraft state cannot be determined at some
     * maneuver date
     */
    public ScheduledManeuver[] setUpManeuvers(final double[] point,
                                              final ManeuverAdapterPropagator propagator)
        throws OrekitException {

        final ScheduledManeuver[] scheduled = new ScheduledManeuver[tunables.length];

        int index = 0;
        for (int i = 0; i < tunables.length; ++i) {
            final TunableManeuver maneuver = tunables[i];
            if (maneuver.isRelativeToPrevious()) {
                // the date of this maneuver is relative to the date of the previous one which
                // has just been set at the previous iteration of this parameters setting loop
                maneuver.setReferenceDate(tunables[i - 1].getDate());
            } else {
                // the date of this maneuver is relative to the cycle start
                final int maneuversPerCycle   = tunables.length / rollingCycles;
                final int cycleIndex          = i / maneuversPerCycle;
                final double offset           = cycleIndex * cycleDuration * Constants.JULIAN_DAY;
                final AbsoluteDate cycleStart = initialState.getDate().shiftedBy(offset);
                maneuver.setReferenceDate(cycleStart);
            }
            for (final SKParameter parameter : maneuver.getParameters()) {
                if (parameter.isTunable()) {
                    parameter.setValue(point[index++]);
                }
            }

            scheduled[i] = maneuver.getManeuver(propagator);
            propagator.addManeuver(scheduled[i].getDate(), scheduled[i].getDeltaV(), scheduled[i].getIsp());

        }

        return scheduled;

    }

    /** {@inheritDoc} */
    public double value(final double[] point) {

        try {

            // set the parameters to the current optimizer-provided values
            final ManeuverAdapterPropagator propagator = new ManeuverAdapterPropagator(reference);
            final ScheduledManeuver[] maneuvers = setUpManeuvers(point, propagator);

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

            // prepare run
            for (final SKControl control : controls) {
                control.initializeRun(maneuvers, propagator,
                                      reference.getMinDate(), reference.getMaxDate(),
                                      rollingCycles);
            }

            // perform propagation
            propagator.propagate(reference.getMinDate().shiftedBy(rollingCycles * cycleDuration * Constants.JULIAN_DAY));

            // compute sum of squared scaled residuals
            double sum = 0;
            for (final SKControl s : controls) {
                final double residual = s.getAchievedValue() - s.getTargetValue();
                final double scaledResidual = residual / s.getScalingDivisor();
                sum += scaledResidual * scaledResidual;
            }

            // return the sum of squared scaled residuals
            return sum;

        } catch (OrekitException oe) {
            return Double.POSITIVE_INFINITY;
        }

    }

}
