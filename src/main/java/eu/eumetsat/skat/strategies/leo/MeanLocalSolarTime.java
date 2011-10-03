/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import org.orekit.errors.PropagationException;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;

import eu.eumetsat.skat.control.SKControl;

/**
 * Station-keeping control attempting to get mean local solar time in a deadband.
 */
public class MeanLocalSolarTime
    implements SKControl, OrekitStepHandler {

    /** Serializable UID. */
    private static final long serialVersionUID = -4929869468941051714L;

    /** {@inheritDoc} */
    public String getName() {
        return "mean local solar time";
    }

    /** {@inheritDoc} */
    public double getTargetValue() {
        // TODO
        return Double.NaN;
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
        // TODO
        return Double.NaN;
    }

    /** {@inheritDoc} */
    public boolean isConstrained() {
        // TODO
        return false;
    }

    /** {@inheritDoc} */
    public double getMin() {
        // TODO
        return Double.NaN;
    }

    /** {@inheritDoc} */
    public double getMax() {
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
