/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;

/**
 * Station-keeping control attempting to keep relative inclination vector between satellites.
 */
public class RelativeInclinationControl extends AbstractSKControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Desired difference in hx. */
    private final double deltaHx;

    /** Desired difference in hy. */
    private final double deltaHy;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Simple constructor.
     * @param name name of the control law
     * @param scalingDivisor divisor to use for scaling the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param referenceName name of the reference spacecraft
     * @param referenceIndex index of the reference spacecraft
     * @param deltaHx desired difference in hx
     * @param deltaHy desired difference in hy
     * @param radius radius of the circle
     * @param samplingStep step to use for sampling throughout propagation
     */
    public RelativeInclinationControl(final String name, final double scalingDivisor,
                                      final String controlledName, final int controlledIndex,
                                      final String referenceName, final int referenceIndex,
                                      final double deltaHx, final double deltaHy,
                                      final double samplingStep) {
        super(name, scalingDivisor, controlledName, controlledIndex, referenceName, referenceIndex,
              0.0, 0.0, Double.POSITIVE_INFINITY);
        this.stephandler  = new Handler();
        this.deltaHx      = deltaHx;
        this.deltaHy      = deltaHy;
        this.samplingStep = samplingStep;
    }

    /** {@inheritDoc} */
    public void initializeRun(ScheduledManeuver[] maneuvers,
                              Propagator propagator, AbsoluteDate start,
                              AbsoluteDate end, int rollingCycles)
        throws OrekitException {
        // TODO
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
        private static final long serialVersionUID = -922603258509726406L;

        /** {@inheritDoc} */
        public void init(final SpacecraftState s0, final AbsoluteDate t) {
            resetLimitsChecks();
        }

        /** {@inheritDoc} */
        public void handleStep(OrekitStepInterpolator interpolator, boolean isLast)
                throws PropagationException {
            // TODO
        }

    }

}
