/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.List;

import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.events.DateDetector;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

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

    /** Cycle end date. */
    private AbsoluteDate cycleEnd;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param propagators propagators to use for each spacecraft
     */
    public Propagation(final int[] spacecraftIndices, final Propagator[] propagators) {
        this.spacecraftIndices = spacecraftIndices.clone();
        this.propagators       = propagators.clone();
    }

    /** Set the end date for current cycle.
     * @param cycleEnd end date for current cycle
     */
    public void setCycleEnd(final AbsoluteDate cycleEnd) {
        this.cycleEnd = cycleEnd;
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException, SkatException {

        final ScenarioState[] updated = new ScenarioState[originals.length];

        // separately propagate each spacecraft
        for (int i = 0; i < spacecraftIndices.length; ++i) {

            // select the current spacecraft affected by this component
            final int index = spacecraftIndices[i];
            final List<ScheduledManeuver> performed = originals[index].getPerformedManeuvers();
            if (performed == null) {
                throw new SkatException(SkatMessages.NO_PERFORMED_MANEUVERS_STATE,
                                        originals[index].getName(), originals[index].getCyclesNumber());
            }

            // set up the propagator with the maneuvers to perform
            propagators[i].clearEventsDetectors();
            for (final ScheduledManeuver maneuver : performed) {
                propagators[i].addEventDetector(new ImpulseManeuver(new DateDetector(maneuver.getDate()),
                                                                    maneuver.getDeltaV(),
                                                                    maneuver.getIsp()));
            }
            propagators[i].setEphemerisMode();

            // perform propagation
            propagators[i].resetInitialState(originals[index].getRealStartState());
            updated[index] = originals[index].updateRealEndState(propagators[i].propagate(cycleEnd));

            // retrieve continuous data
            updated[index] = updated[index].updatePerformedEphemeris(propagators[i].getGeneratedEphemeris());

        }

        return updated;

    }

}
