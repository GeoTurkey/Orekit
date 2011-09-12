/* Copyright 2011 Eumetsat */

package eu.eumetsat.skat.control.strategies.leo;

import org.orekit.errors.PropagationException;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;

import eu.eumetsat.skat.control.StationKeepingGoal;

/**
 * Station-keeping goal attempting to follow a specified ground-track
 * at a specified latitude.
 */
public class GroundTrackGrid
    implements StationKeepingGoal, OrekitStepHandler {

    /** Serializable UID. */
    private static final long serialVersionUID = 2042987246037436750L;

    /** {@inheritDoc} */
    public double getTarget() {
        // TODO
        return Double.NaN;
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
        // TODO
        return Double.NaN;
    }

    /** {@inheritDoc} */
    public void handleStep(OrekitStepInterpolator interpolator, boolean isLast)
        throws PropagationException {
        // TODO
    }

    /** {@inheritDoc} */
    public void reset() {
        // TODO
    }

}
