/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import org.apache.commons.math.random.RandomGenerator;
import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Class for splitting large maneuvers as several smaller ones.
 * @author Luc Maisonobe
 */
public class ManeuverSplitter implements ScenarioComponent {

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Indicator for applying this error to in-plane maneuvers. */
    private final boolean inPlane;

    /** Indicator for applying this error to out-of-plane maneuvers. */
    private final boolean outOfPlane;

    /** Maximum allowed velocity increment. */
    private final double maxDV;

    /** Minimum time between split parts. */
    private final double minDT;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param inPlane if true, the error applies to in-plane maneuvers
     * @param outOfPlane if true, the error applies to out-of-plane maneuvers
     * @param maxDV maximum allowed velocity increment
     * @param minDT minimum time between split parts
     */
    public ManeuverSplitter(final int[] spacecraftIndices,
                            final boolean inPlane, final boolean outOfPlane,
                            final double maxDV, final double minDT)
                                    throws IllegalArgumentException {
        this.spacecraftIndices = spacecraftIndices.clone();
        this.inPlane           = inPlane;
        this.outOfPlane        = outOfPlane;
        this.maxDV             = maxDV;
        this.minDT             = minDT;
    }

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
