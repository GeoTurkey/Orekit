package eu.eumetsat.skat.realization;

import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;

/**
 * Class for splitting large maneuvers as several smaller ones.
 * @author Luc Maisonobe
 */
public class ManeuverSplitter implements ScenarioComponent {

    /** {@inheritDoc} */
    public ScenarioState updateState(final ScenarioState initialState, final AbsoluteDate target)
        throws OrekitException {
        // TODO Auto-generated method stub
        return null;
    }

}
