package eu.eumetsat.skat.realization;

import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;

/**
 * Class for introducing random realization errors to maneuvers.
 * @author Luc Maisonobe
 */
public class ManeuverRealizationError implements ScenarioComponent {

    /** {@inheritDoc} */
    public ScenarioState updateState(final ScenarioState original, AbsoluteDate target)
        throws OrekitException {
        // TODO
        return null;
    }

}
