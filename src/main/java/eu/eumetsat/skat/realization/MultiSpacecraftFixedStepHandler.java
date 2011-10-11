/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.orekit.errors.PropagationException;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventsLogger.LoggedEvent;
import org.orekit.propagation.sampling.OrekitFixedStepHandler;
import org.orekit.time.AbsoluteDate;

/**
 * Fixed step handler managing several spacecraft at a time.
 * <p>
 * This step handler is driven by propagation of a master spacecraft,
 * and allows to compute inter-satellite data by synchronizing slave
 * spacecrafts to the master one. All spacecraft propagations are done
 * using ephemerides, the master spacecraft being the first one in
 * the list.
 * </p>
 * @author Luc Maisonobe
 */
class MultiSpacecraftFixedStepHandler implements OrekitFixedStepHandler {

    /** Serializable UID. */
    private static final long serialVersionUID = 6043054895494909500L;

    /** Spacecrafts ephemerides (master spacecraft is at index 0). */
    private final List<BoundedPropagator> ephemerides;

    /** Maneuver events. */
    private final List<List<LoggedEvent>> events;

    /** Ascending nodes dates. */
    private final List<AbsoluteDate> ascendingNodeDates;

    /** Ascending nodes solar times. */
    private final List<Double> ascendingNodesSolarTimes;

    /** Descending nodes dates. */
    private final List<AbsoluteDate> descendingNodeDates;

    /** Descending nodes solar times. */
    private final List<Double> descendingNodesSolarTimes;

    /** Simple constructor.
     * @param ephemerides spacecraft ephemerides (master spacecraft is at index 0)
     * @param events maneuver events
     */
    public MultiSpacecraftFixedStepHandler(final List<BoundedPropagator> ephemerides,
                                           final List<List<LoggedEvent>> events) {
        this.ephemerides               = ephemerides;
        this.events                    = events;
        this.ascendingNodeDates        = new ArrayList<AbsoluteDate>();
        this.ascendingNodesSolarTimes  = new ArrayList<Double>();
        this.descendingNodeDates       = new ArrayList<AbsoluteDate>();
        this.descendingNodesSolarTimes = new ArrayList<Double>();
        for (int i = 0; i < ephemerides.size(); ++i) {
            ascendingNodeDates.add(AbsoluteDate.PAST_INFINITY);
            descendingNodeDates.add(AbsoluteDate.PAST_INFINITY);
        }
    }

    /** {@inheritDoc} */
    public void handleStep(SpacecraftState currentState, boolean isLast)
            throws PropagationException {
        // TODO Auto-generated method stub
    }

    /** Find the closest node.
     * @param ephemeride spacecraft ephemerides
     * @param date current date
     * @return spacecraft state at node
     */
    private SpacecraftState findClosestNode(final BoundedPropagator ephemeride, final AbsoluteDate date) {
        // TODO
        return null;
    }

}
