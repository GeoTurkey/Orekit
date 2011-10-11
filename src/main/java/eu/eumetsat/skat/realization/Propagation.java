/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.utils.Monitorable;
import eu.eumetsat.skat.utils.MonitorableKey;

/**
 * Class for simple propagation of a station-keeping cycle.
 * <p>
 * This class performs propagation of real spacecraft state, using
 * the maneuver that have been determined in the control part of the
 * simulation.
 * </p>
 * @author Luc Maisonobe
 */
public class Propagation implements ScenarioComponent {

    /** Orbit propagator. */
    private final Propagator propagator;

    /** Monitored values. */
    private final List<Monitorable> monitorables;

    /** Simple constructor.
     * @param propagator propagator to use
     */
    public Propagation(final Propagator propagator) {
        this.propagator   = propagator;
        this.monitorables = new ArrayList<Monitorable>();
    }

    /** Monitor a station-keeping data.
     * @param key key of station-keeping data to monitor
     */
    public void monitor(final MonitorableKey key) {
        // TODO
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals, final AbsoluteDate target)
        throws OrekitException {

        ScenarioState[] updated = new ScenarioState[originals.length];

        for (int i = 0; i < originals.length; ++i) {

            // set up the propagator with the maneuvers to perform
            propagator.clearEventsDetectors();
            for (final ImpulseManeuver maneuver : originals[i].getTheoreticalManeuvers()) {
                propagator.addEventDetector(maneuver);
            }
            propagator.setMasterMode(new Handler());

            // perform propagation
            propagator.resetInitialState(originals[i].getRealState());
            SpacecraftState finalState = propagator.propagate(target);

            // build the updated scenario state
            updated[i] = new ScenarioState(finalState, finalState,
                                           new ArrayList<ImpulseManeuver>(),
                                           new ArrayList<ImpulseManeuver>());

        }

        return updated;

    }

    /** Inner class for monitoring. */
    private class Handler implements OrekitStepHandler {

        /** {@inheritDoc} */
        public void handleStep(final OrekitStepInterpolator interpolator, final boolean isLast)
            throws PropagationException {
            // TODO Auto-generated method stub
        }

        /** {@inheritDoc} */
        public void reset() {
        }
        
    }

}
