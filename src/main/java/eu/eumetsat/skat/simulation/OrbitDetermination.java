package eu.eumetsat.skat.simulation;

import org.orekit.errors.OrekitException;
import org.orekit.propagation.SpacecraftState;

import eu.eumetsat.skat.CycleComponent;

/**
 * Class for orbit-determination simulation.
 * <p>
 * This class simulate orbit determination. It starts from the real orbit
 * and compute an estimated orbit.
 * </p>
 * @author Luc Maisonobe
 */
public class OrbitDetermination implements CycleComponent {

    /** {@inheritDoc} */
    public SpacecraftState run(final SpacecraftState initialState, final double duration)
        throws OrekitException {
        // TODO Auto-generated method stub
        return null;
    }

}
