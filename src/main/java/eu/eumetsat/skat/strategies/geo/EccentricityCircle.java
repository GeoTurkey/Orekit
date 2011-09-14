/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import org.orekit.errors.PropagationException;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;

import eu.eumetsat.skat.control.SKControl;

/**
 * Station-keeping control attempting to follow a specified eccentricity circle.
 */
public class EccentricityCircle
    implements SKControl, OrekitStepHandler {

    /** Serializable UID. */
    private static final long serialVersionUID = 2605474503798176086L;

    /** {@inheritDoc} */
    public String getName() {
        return "eccentricity circle";
    }

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
