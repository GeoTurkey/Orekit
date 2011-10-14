/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.events.DateDetector;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;

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

    /** Simple constructor.
     */
    public Propagation(final Propagator propagator) {
        this.propagator = propagator;
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals, final AbsoluteDate target)
        throws OrekitException {

        final ScenarioState[] updated = new ScenarioState[originals.length];

        // separately propagate each spacecraft
        for (int i = 0; i < originals.length; ++i) {

            // set up the propagator with the maneuvers to perform
            propagator.clearEventsDetectors();
            for (final ScheduledManeuver maneuver : originals[i].getTheoreticalManeuvers()) {
                propagator.addEventDetector(new ImpulseManeuver(new DateDetector(maneuver.getDate()),
                                                                maneuver.getDeltaV(),
                                                                maneuver.getIsp()));
            }
            propagator.setEphemerisMode();

            // perform propagation
            propagator.resetInitialState(originals[i].getRealStartState());
            updated[i] = originals[i].updateRealEndState(propagator.propagate(target));

            // retrieve continuous data
            updated[i] = originals[i].updateEphemeris(propagator.getGeneratedEphemeris());

        }

        return updated;

    }

}
