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

    /** Update a scenario state by applying component process.
     * <p>
     * The exact meaning of the date depends on component. For orbit
     * determination component, it may be the current date. For propagation
     * it may be a target date. For control loop it may be end of station keeping
     * cycle.
     * </p>
     * @param original original state of the scenario
     * @param date date of the update (it's exact meaning depends
     * on component)
     * @return updated state
     * @exception OrekitException if simulation cannot be performed
     */
    ScenarioState updateState(ScenarioState original, AbsoluteDate target)
        throws OrekitException;

}
