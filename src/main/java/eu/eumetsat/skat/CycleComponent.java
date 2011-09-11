/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat;

import org.orekit.errors.OrekitException;
import org.orekit.propagation.SpacecraftState;

/**
 * Interface representing a component of a station-keeping cycle.
 * @author Luc Maisonobe
 */
public interface CycleComponent {

    /** Run the cycle component.
     * @param initialState initial state of the cycle
     * @param duration cycle duration
     * @return achieved state
     * @exception OrekitException if simulation cannot be performed
     */
    SpacecraftState run(SpacecraftState initialState, double duration)
        throws OrekitException;

}
