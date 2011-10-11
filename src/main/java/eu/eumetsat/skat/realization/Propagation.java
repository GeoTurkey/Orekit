/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventsLogger;
import org.orekit.propagation.events.EventsLogger.LoggedEvent;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.utils.MonitorableSKData;

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

    /** Output step for monitoring. */
    private final double outputstep;

    /** Monitored values. */
    private final List<MonitorableSKData> monitorables;

    /** Simple constructor.
     * @param propagator propagator to use
     * @param outputStep output step for monitoring (s)
     */
    public Propagation(final Propagator propagator, final double outputStep) {
        this.propagator   = propagator;
        this.outputstep   = outputStep;
        this.monitorables = new ArrayList<MonitorableSKData>();
    }

    /** Monitor a station-keeping data.
     * @param key key of station-keeping data to monitor
     */
    public void monitor(final MonitorableSKData key) {
        monitorables.add(key);
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals, final AbsoluteDate target)
        throws OrekitException {

        final ScenarioState[] updated = new ScenarioState[originals.length];
        final List<BoundedPropagator> ephemerides = new ArrayList<BoundedPropagator>(originals.length);
        final List<List<LoggedEvent>> events      = new ArrayList<List<LoggedEvent>>(originals.length);


        // separately propagate each spacecraft
        for (int i = 0; i < originals.length; ++i) {

            final EventsLogger logger = new EventsLogger();

            // set up the propagator with the maneuvers to perform
            propagator.clearEventsDetectors();
            for (final ImpulseManeuver maneuver : originals[i].getTheoreticalManeuvers()) {
                propagator.addEventDetector(logger.monitorDetector(maneuver));
            }
            propagator.setEphemerisMode();

            // perform propagation
            propagator.resetInitialState(originals[i].getRealState());
            SpacecraftState finalState = propagator.propagate(target);
            events.add(logger.getLoggedEvents());
            ephemerides.add(propagator.getGeneratedEphemeris());

            // build the updated scenario state
            updated[i] = new ScenarioState(finalState, finalState,
                                           new ArrayList<ImpulseManeuver>(),
                                           new ArrayList<ImpulseManeuver>());

        }

        // monitor station-keeping data from all spacecrafts together
        // (this ensures we can compute inter-satellite data)
        ephemerides.get(0).setMasterMode(outputstep,
                                         new MultiSpacecraftFixedStepHandler(ephemerides, events));
        ephemerides.get(0).propagate(target);

        return updated;

    }

}
