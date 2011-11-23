/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies;

import org.orekit.propagation.Propagator;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.AbstractSKControl;

/**
 * Station-keeping control law minimizing maneuvers velocity increments.
 * <p>
 * This control value is:
 * <pre>
 *   &sum;|&Delta;V<sub>i</sub>|
 * </pre>
 * where &Delta;V<sub>i</sub> is the maneuver increment for maneuver number i.
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to 0 attempts to minimize maneuvers.
 * </p>
 * @author Luc Maisonobe
 */
public class MinimizedManeuvers extends AbstractSKControl {

    /** Indicator for counting in-plane maneuvers. */
    private final boolean inPlane;

    /** Indicator for counting out-of-plane maneuvers. */
    private final boolean outOfPlane;

    /** Cumulated velocity increments. */
    private double sumDeltaV;

    /** Simple constructor.
     * @param name name of the control law
     * @param scalingDivisor divisor to use for scaling the control law
     * @param controlled name of the controlled spacecraft
     * @param inPlane if true, in-plane maneuvers are taken into account
     * @param outOfPlane if true, out-of-plane maneuvers are taken into account
     */
    public MinimizedManeuvers(final String name, final double scalingDivisor,
                              final String controlled,
                              final boolean inPlane, final boolean outOfPlane) {
        super(name, scalingDivisor, controlled, null, 0.0, 0.0, Double.POSITIVE_INFINITY);
        this.inPlane    = inPlane;
        this.outOfPlane = outOfPlane;
    }

    /** {@inheritDoc} */
    @Override
    public void initializeRun(final ScheduledManeuver[] maneuvers,
                              final Propagator propagator,
                              final AbsoluteDate start, final AbsoluteDate end) {
        sumDeltaV = 0;
        for (final ScheduledManeuver maneuver : maneuvers) {
            if ((inPlane && maneuver.isInPlane()) || (outOfPlane && !(maneuver.isInPlane()))) {
                sumDeltaV += maneuver.getDeltaV().getNorm();
            }
        }
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
        return sumDeltaV;
    }

    /** {@inheritDoc} */
    public EventDetector getEventDetector() {
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return null;
    }

}
