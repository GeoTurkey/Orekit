/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.apache.commons.math.analysis.MultivariateRealFunction;
import org.orekit.errors.PropagationException;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;

/** Objective function to be optimizedby the {@link ControlLoop}. */
class ObjectiveFunction implements MultivariateRealFunction, OrekitStepHandler {

    /** Serializable UID. */
    private static final long serialVersionUID = 1547228582323575520L;

    /** Orbit propagator. */
    private final Propagator propagator;

    /** Station-keeping controls. */
    private final List<ScaledControl> controls;

    /** Tunable control parameters. */
    private final List<SKParameter> parameters;

    /** Target date for end of cycle. */
    private final AbsoluteDate cycleEnd;

    /** Initial state. */
    private final SpacecraftState initialState;

    /** Maneuvers that are already scheduled. */
    private final List<ImpulseManeuver> scheduledManeuvers;

    /** Active event detectors. */
    private final Set<EventDetector> detectors;

    /** Active step handlers. */
    private final Set<OrekitStepHandler> handlers;

    /** Simple constructor.
     * @param propagator propagator to use
     * @param parameters station-keeping parameters
     * @param controls station-keeping controls
     * @param cycleEnd target date for end of cycle
     * @param initialState initial state
     * @param scheduledManeuvers maneuvers that are already scheduled
     * (and hence not optimized themselves)
     */
    public ObjectiveFunction(final Propagator propagator,
                             final List<SKParameter> parameters,
                             final List<ScaledControl> controls,
                             final AbsoluteDate cycleEnd,
                             final SpacecraftState initialState,
                             final List<ImpulseManeuver> scheduledManeuvers) {

        this.propagator         = propagator;
        this.parameters         = parameters;
        this.controls           = controls;
        this.cycleEnd           = cycleEnd;
        this.initialState       = initialState;
        this.scheduledManeuvers = scheduledManeuvers;

        // different station-keeping elements may be associated with the same detector,
        // so we must gather them in a set to avoid notifying them several times
        detectors = new HashSet<EventDetector>();

        // get the detectors associated with parameters
        for (final SKParameter parameter : parameters) {
            if (parameter.getEventDetector() != null) {
                detectors.add(parameter.getEventDetector());
            }
        }

        // get the detectors associated with control laws
        for (final ScaledControl sc : controls) {
            if (sc.getControl().getEventDetector() != null) {
                detectors.add(sc.getControl().getEventDetector());
            }
        }

        // different station-keeping elements may be associated with the same handler,
        // so we must gather them in a set to avoid notifying them several times
        handlers = new HashSet<OrekitStepHandler>();

        // get the step handlers associated with parameters
        for (final SKParameter parameter : parameters) {
            if (parameter.getStepHandler() != null) {
                handlers.add(parameter.getStepHandler());
            }
        }

        // get the step handlers associated with control laws
        for (final ScaledControl sc : controls) {
            if (sc.getControl().getStepHandler() != null) {
                handlers.add(sc.getControl().getStepHandler());
            }
        }

    }

    /** {@inheritDoc} */
    public double value(final double[] point) {

        try {
            // set the parameters to the current test values
            for (int i = 0; i < point.length; ++i) {
                parameters.get(i).setValue(point[i]);
            }


            // set up the propagator with the station-keeping elements
            // that are part of the optimization process in the control loop
            propagator.clearEventsDetectors();
            for (final EventDetector detector : detectors) {
                propagator.addEventDetector(detector);
            }
            propagator.setMasterMode(this);

            // set up the scheduled maneuvers that are not optimized
            for (final ImpulseManeuver maneuver : scheduledManeuvers) {
                propagator.addEventDetector(maneuver);
            }

            // perform propagation
            propagator.resetInitialState(initialState);
            propagator.propagate(cycleEnd);

            // compute sum of squared scaled residuals
            double sum = 0;
            for (final ScaledControl s : controls) {
                sum += s.getScaledResidualSquared();
            }

            return sum;
        } catch (PropagationException pe) {
            return Double.POSITIVE_INFINITY;
        }

    }

    /** {@inheritDoc} */
    public void reset() {
        for (final OrekitStepHandler handler : handlers) {
            handler.reset();
        }
    }

    /** {@inheritDoc} */
    public void handleStep(final OrekitStepInterpolator interpolator, final boolean isLast)
        throws PropagationException {
        for (final OrekitStepHandler handler : handlers) {
            handler.handleStep(interpolator, isLast);
        }
    }

}
