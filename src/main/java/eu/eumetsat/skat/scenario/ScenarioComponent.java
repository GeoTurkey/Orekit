/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.scenario;

import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.utils.SkatException;

/**
 * Interface representing a component of a station-keeping scenario.
 * <p>
 * Station-keeping scenario components are the basic building blocks
 * used to build scenarii. They can be elementary components like
 * an orbit determination phase, or a free-flying phase or they
 * can be composed components like a complete cycle including
 * the aforementioned elementary components.
 * </p>
 * @author Luc Maisonobe
 */
public interface ScenarioComponent {

    /** Set the end date for current cycle.
     * @param cycleEnd end date for current cycle
     */
    void setCycleEnd(AbsoluteDate cycleEnd);

    /** Update a scenario state by applying component process.
     * @param originals original states of the scenario
     * @return updated state
     * @exception OrekitException if simulation cannot be performed
     * @exception SkatException if some data is missing in the originals states
     */
    ScenarioState[] updateStates(ScenarioState[] originals)
        throws OrekitException, SkatException;

}
