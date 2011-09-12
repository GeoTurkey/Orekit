/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.scenario;

import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

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

    /** Apply the scenario component.
     * @param original original state of the scenario
     * @param target target date for end of simulation
     * @return achieved state
     * @exception OrekitException if simulation cannot be performed
     */
    ScenarioState apply(ScenarioState original, AbsoluteDate target)
        throws OrekitException;

}
