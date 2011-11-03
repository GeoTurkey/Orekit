/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;

/**
 * Class for computing cross-coupling in maneuvers.
 * @author Luc Maisonobe
 */
public class ManeuverCrossCoupling implements ScenarioComponent {

    /** {@inheritDoc} */
    public void setCycleEnd(final AbsoluteDate cycleEnd) {
        // nothing to do here
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException {
        // TODO Auto-generated method stub
        return null;
    }

}
