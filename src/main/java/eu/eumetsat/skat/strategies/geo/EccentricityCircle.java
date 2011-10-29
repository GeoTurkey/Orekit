/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import org.orekit.errors.PropagationException;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;

import eu.eumetsat.skat.control.AbstractSKControl;

/**
 * Station-keeping control attempting to follow a specified eccentricity circle.
 */
public class EccentricityCircle extends AbstractSKControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Abscissa of the circle center. */
    private final double centerX;

    /** Ordinate of the circle center. */
    private final double centerY;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Simple constructor.
     * @param name name of the control law
     * @param scale of the control law
     * @param controlled name of the controlled spacecraft
     * @param centerX abscissa of the circle center
     * @param centerY ordinate of the circle center
     * @param radius radius of the circle
     * @param samplingStep step to use for sampling throughout propagation
     */
    public EccentricityCircle(final String name, final double scale, final String controlled,
                              final double centerX, final double centerY, final double radius,
                              final double samplingStep) {
        super(name, scale, controlled, null, radius, 0.0, Double.POSITIVE_INFINITY);
        this.stephandler  = new Handler();
        this.centerX      = centerX;
        this.centerY      = centerY;
        this.samplingStep = samplingStep;
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
        // TODO
        return Double.NaN;
    }

    /** {@inheritDoc} */
    public EventDetector getEventDetector() {
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return stephandler;
    }

    /** Inner class for step handling. */
    private class Handler implements OrekitStepHandler {

        /** Serializable UID. */
        private static final long serialVersionUID = 220407581859026265L;

        /** {@inheritDoc} */
        public void reset() {
            // TODO
        }

        /** {@inheritDoc} */
        public void handleStep(OrekitStepInterpolator interpolator, boolean isLast)
                throws PropagationException {
            // TODO
        }

    }

}
