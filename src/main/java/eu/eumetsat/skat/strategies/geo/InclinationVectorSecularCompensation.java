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

import eu.eumetsat.skat.control.AbstractSKMonoControl;

/**
 * Station-keeping control attempting to compensate inclination secular evolution.
 * @author Luc Maisonobe
 */
public class InclinationVectorSecularCompensation extends AbstractSKMonoControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Abscissa of target inclination vector. */
    private final double hx0;

    /** Ordinate of target inclination vector. */
    private final double hy0;

    /** Maximal offset from target. */
    private double maxDelta;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Simple constructor.
     * @param name name of the control law
     * @param scale of the control law
     * @param hx0 abscissa of target inclination vector
     * @param hy0 ordinate of target inclination vector
     * @param samplingStep step to use for sampling throughout propagation
     */
    public InclinationVectorSecularCompensation(final String name, final double scale,
                                                final double hx0, final double hy0,
                                                final double samplingStep) {
        super(name, scale, 0.0);
        this.stephandler  = new Handler();
        this.hx0          = hx0;
        this.hy0          = hy0;
        this.samplingStep = samplingStep;
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
                    double dx = state.getHx() - hx0;
                    double dy = state.getHy() - hy0;
                    maxDelta = FastMath.max(maxDelta, FastMath.hypot(dx, dy));

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
