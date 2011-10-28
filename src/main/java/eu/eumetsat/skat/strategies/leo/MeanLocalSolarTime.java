/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import org.orekit.errors.PropagationException;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;

import eu.eumetsat.skat.control.AbstractSKMonoControl;

/**
 * Station-keeping control attempting to get mean local solar time in a deadband.
 */
public class MeanLocalSolarTime extends AbstractSKMonoControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Simple constructor.
     * @param name name of the control law
     * @param scale of the control law
     * @param solarTime target solar time
     * @param samplingStep step to use for sampling throughout propagation
     */
    public MeanLocalSolarTime(final String name, final double scale,
                              final double solarTime, final double samplingStep) {
        super(name, scale, solarTime);
        this.stephandler  = new Handler();
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
    public EventDetector getEventDetector() {
        // TODO
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return stephandler;
    }

    /** Inner class for step handling. */
    private class Handler implements OrekitStepHandler {

        /** Serializable UID. */
        private static final long serialVersionUID = 392097837477401303L;

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
