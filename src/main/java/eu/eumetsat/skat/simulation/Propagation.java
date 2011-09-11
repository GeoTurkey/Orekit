package eu.eumetsat.skat.simulation;

import org.orekit.errors.OrekitException;
import org.orekit.propagation.SpacecraftState;

import eu.eumetsat.skat.CycleComponent;

/**
 * Class for simple propagation of a station-keeping cycle.
 * <p>
 * This class performs propagation of real spacecraft state, using
 * the maneuver that have been determined in the control part of the
 * simulation.
 * </p>
 * @author Luc Maisonobe
 */
public class Propagation implements CycleComponent {

    /** {@inheritDoc} */
    public SpacecraftState run(final SpacecraftState initialState, final double duration)
        throws OrekitException {
        // TODO Auto-generated method stub
        return null;
    }

}
