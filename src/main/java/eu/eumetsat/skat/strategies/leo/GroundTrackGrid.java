/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import org.orekit.errors.PropagationException;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;

import eu.eumetsat.skat.control.AbstractSKControl;

/**
 * Station-keeping control attempting to follow a specified ground-track
 * at a specified latitude.
 */
public class GroundTrackGrid extends AbstractSKControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Reference point latitude. */
    private final double latitude;

    /** Reference point longitude. */
    private final double longitude;

    /** Simple constructor.
     * @param name name of the control law
     * @param scale of the control law
     * @param controlled name of the controlled spacecraft
     * @param latitude reference point latitude
     * @param longitude reference point longitude
     * @param samplingStep step to use for sampling throughout propagation
     */
    public GroundTrackGrid(final String name, final double scale,
                           final String controlled,
                           final double latitude, final double longitude,
                           final double samplingStep) {
        super(name, scale, controlled, null, 0.0,
              Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        this.stephandler = new Handler();
        this.latitude    = latitude;
        this.longitude   = longitude;
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
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
        private static final long serialVersionUID = -57789560517985646L;

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
