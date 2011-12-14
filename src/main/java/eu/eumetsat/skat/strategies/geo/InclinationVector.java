/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.stat.descriptive.rank.Median;
import org.apache.commons.math.util.FastMath;
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
 * Station-keeping control for inclination vector.
 * <p>
 * This control value is:
 * <pre>
 *   &radic; [(median(h<sub>x</sub>) - c<sub>x</sub>)<sup>2</sup> + (median(h<sub>y</sub>) - c<sub>y</sub>)<sup>2</sup>]
 * </pre>
 * where (h<sub>x</sub>, h<sub>y</sub>) is the inclination vector ans
 * (c<sub>x</sub>, c<sub>y</sub>) a reference point.
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to 0 attempts to have the inclination vector move in a domain centered
 * around the reference point (c<sub>x</sub>, c<sub>y</sub>).
 * </p>
 * <p>
 * Using a median instead of a mean improves robustness with respect to
 * outliers, which occur when starting far from the desired window.
 * </p>
 * @author Luc Maisonobe
 */
public class InclinationVector extends AbstractSKControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Abscissa of target inclination vector. */
    private final double targetHx;

    /** Ordinate of target inclination vector. */
    private final double targetHy;

    /** Sample of inclination x component during station keeping cycle. */
    private List<Double> sampleX;

    /** Sample of inclination y component during station keeping cycle. */
    private List<Double> sampleY;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Simple constructor.
     * @param name name of the control law
     * @param scalingDivisor divisor to use for scaling the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param targetHx abscissa of target inclination vector
     * @param targetHy ordinate of target inclination vector
     * @param circleRadius limit circle radius
     * @param samplingStep step to use for sampling throughout propagation
     */
    public InclinationVector(final String name, final double scalingDivisor,
                             final String controlledName, final int controlledIndex,
                             final double targetHx, final double targetHy,
                             final double circleRadius, final double samplingStep) {
        super(name, scalingDivisor, controlledName, controlledIndex, null, -1, 0.0, 0.0, circleRadius);
        this.stephandler  = new Handler();
        this.targetHx     = targetHx;
        this.targetHy     = targetHy;
        this.samplingStep = samplingStep;
        this.sampleX      = new ArrayList<Double>();
        this.sampleY      = new ArrayList<Double>();
    }

    /** {@inheritDoc} */
    public void initializeRun(final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, AbsoluteDate start, AbsoluteDate end, int rollingCycles) {
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {

        if (limitsExceeded() > 0) {
            // we escaped the limit circle during the cycle, we need to adjust the maneuvers

            // compute center of the motion along the x axis
            final double[] dataX = new double[sampleX.size()];
            for (int i = 0; i < dataX.length; ++i) {
                dataX[i] = sampleX.get(i);
            }
            final double medianX = new Median().evaluate(dataX);

            // compute center of the motion along the y axis
            final double[] dataY = new double[sampleY.size()];
            for (int i = 0; i < dataY.length; ++i) {
                dataY[i] = sampleY.get(i);
            }
            final double medianY = new Median().evaluate(dataY);

            // compute objective function value
            return FastMath.hypot(medianX - targetHx, medianY - targetHy);

        } else {
            // we stay within the circle, don't bother changing the maneuvers
            return 0;
        }

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
        public void init(final SpacecraftState s0, final AbsoluteDate t) {
            resetLimitsChecks();
            sampleX.clear();
            sampleY.clear();
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
                for (AbsoluteDate date = minDate; date.compareTo(maxDate) < 0; date = date.shiftedBy(samplingStep)) {

                    // compute position in Earth frame
                    interpolator.setInterpolatedDate(date);
                    final SpacecraftState state = interpolator.getInterpolatedState();

                    // add inclination vector to sample
                    sampleX.add(state.getHx());
                    sampleY.add(state.getHy());

                    // check limit circle violations
                    checkLimits(FastMath.hypot(state.getHx() - targetHx, state.getHy() - targetHy));

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
