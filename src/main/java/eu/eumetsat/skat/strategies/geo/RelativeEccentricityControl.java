/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import org.orekit.errors.PropagationException;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;

import eu.eumetsat.skat.control.SKControl;

/**
 * Station-keeping control attempting to keep relative eccentricity vector between satellites.
 */
public class RelativeEccentricityControl implements SKControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Name of the control law. */
    private final String name;

    /** Scale of the control law. */
    private final double scale;

    /** Desired difference in ex. */
    private final double deltaEx;

    /** Desired difference in ey. */
    private final double deltaEy;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Simple constructor.
     * @param name name of the control law
     * @param scale of the control law
     * @param deltaEx desired difference in ex
     * @param deltaEy desired difference in ey
     * @param radius radius of the circle
     * @param samplingStep step to use for sampling throughout propagation
     */
    public RelativeEccentricityControl(final String name, final double scale,
                                       final double deltaEx, final double deltaEy,
                                       final double samplingStep) {
        this.stephandler  = new Handler();
        this.name         = name;
        this.scale        = scale;
        this.deltaEx      = deltaEx;
        this.deltaEy      = deltaEy;
        this.samplingStep = samplingStep;
    }

    /** {@inheritDoc} */
    public String getName() {
        return name;
    }

    /** {@inheritDoc} */
    public double getScale() {
        return scale;
    }

    /** {@inheritDoc} */
    public double getTargetValue() {
        return 0.0;
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
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return stephandler;
    }

    /** Inner class for step handling. */
    private class Handler implements OrekitStepHandler {

        /** Serializable UID. */
        private static final long serialVersionUID = 7044667663692366684L;

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
