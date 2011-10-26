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

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Orbit propagators. */
    private final Propagator[] propagators;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param propagators propagators to use for each spacecraft
     */
    public Propagation(final int[] spacecraftIndices, final Propagator[] propagators) {
        this.spacecraftIndices = spacecraftIndices.clone();
        this.propagators       = propagators.clone();
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals, final AbsoluteDate target)
        throws OrekitException {

        final ScenarioState[] updated = new ScenarioState[originals.length];

        // separately propagate each spacecraft
        for (int i = 0; i < spacecraftIndices.length; ++i) {

            // select the current spacecraft affected by this component
            final int index = spacecraftIndices[i];

            // set up the propagator with the maneuvers to perform
            propagators[i].clearEventsDetectors();
            for (final ScheduledManeuver maneuver : originals[index].getTheoreticalManeuvers()) {
                propagators[i].addEventDetector(new ImpulseManeuver(new DateDetector(maneuver.getDate()),
                                                                    maneuver.getDeltaV(),
                                                                    maneuver.getIsp()));
            }
            propagators[i].setEphemerisMode();

            // perform propagation
            propagators[i].resetInitialState(originals[index].getRealStartState());
            updated[index] = originals[index].updateRealEndState(propagators[i].propagate(target));

            // retrieve continuous data
            updated[index] = originals[index].updateEphemeris(propagators[i].getGeneratedEphemeris());

        }

        return updated;

    }

}
