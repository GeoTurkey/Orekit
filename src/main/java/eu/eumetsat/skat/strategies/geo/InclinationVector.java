/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import org.apache.commons.math.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.AbstractSKControl;

/**
 * Station-keeping control for inclination vector.
 * @author Luc Maisonobe
 */
public class InclinationVector extends AbstractSKControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Abscissa of target inclination vector. */
    private final double targetHx;

    /** Ordinate of target inclination vector. */
    private final double targetHy;

    /** Maximal offset from target. */
    private double maxDelta;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Simple constructor.
     * @param name name of the control law
     * @param scale of the control law
     * @param controlled name of the controlled spacecraft
     * @param targetHx abscissa of target inclination vector
     * @param targetHy ordinate of target inclination vector
     * @param samplingStep step to use for sampling throughout propagation
     */
    public InclinationVector(final String name, final double scale,
                             final String controlled,
                             final double targetHx, final double targetHy,
                             final double samplingStep) {
        super(name, scale, controlled, null, 0.0, 0.0, Double.POSITIVE_INFINITY);
        this.stephandler  = new Handler();
        this.targetHx     = targetHx;
        this.targetHy     = targetHy;
        this.samplingStep = samplingStep;
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
        return maxDelta;
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
        private static final long serialVersionUID = 8803174499877772678L;

        /** {@inheritDoc} */
        public void reset() {
            // set the initial values at infinite, to make sure they will be updated
            // properly as soon as simulation starts
            maxDelta = Double.NEGATIVE_INFINITY;
        }

        /** {@inheritDoc} */
        public void handleStep(OrekitStepInterpolator interpolator, boolean isLast)
            throws PropagationException {

            try {

                // find step boundaries
                final AbsoluteDate minDate =
                        interpolator.isForward() ? interpolator.getPreviousDate() : interpolator.getCurrentDate();
                final AbsoluteDate maxDate =
                        interpolator.isForward() ? interpolator.getCurrentDate() : interpolator.getPreviousDate();

                // loop throughout step
                for (AbsoluteDate date = minDate;
                        date.compareTo(maxDate) < 0;
                        date = date.shiftedBy(samplingStep)) {

                    // compute position in Earth frame
                    interpolator.setInterpolatedDate(date);
                    final SpacecraftState state = interpolator.getInterpolatedState();

                    // update inclination excursion
                    final double dx = state.getHx() - targetHx;
                    final double dy = state.getHy() - targetHy;
                    maxDelta        = FastMath.max(maxDelta, FastMath.hypot(dx, dy));

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
