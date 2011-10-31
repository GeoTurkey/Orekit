/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.AbstractSKControl;

/**
 * Station-keeping control attempting to center East-West longitude
 * excursion around a specified center throughout a cycle.
 * <p>
 * This control value is:
 * <pre>
 *   (max(l(t) + min(l(t)) / 2
 * </pre>
 * where l(t) is the spacecraft longitude at time t and the min and max
 * functions are evaluated for the complete cycle duration.
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to l<sub>c</sub> attempts to have a longitude excursion covered by the
 * satellite centered around the l<sub>c</sub> longitude during the
 * station-keeping, the same margin being available on both sides.
 * </p>
 * @author Luc Maisonobe
 */
public class CenteredLongitude extends AbstractSKControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Earth model to use to compute longitudes. */
    private final BodyShape earth;

    /** Minimal longitude (more westward) reached during station keeping cycle. */
    private double minL;

    /** Maximal longitude (more eastward) reached during station keeping cycle. */
    private double maxL;

    /** Simple constructor.
     * @param name name of the control law
     * @param scale of the control law
     * @param controlled name of the controlled spacecraft
     * @param center longitude slot center
     * @param samplingStep step to use for sampling throughout propagation
     * @param earth Earth model to use to compute longitudes
     */
    public CenteredLongitude(final String name, final double scale,
                                final String controlled,
                                final double center, final double samplingStep,
                                final BodyShape earth) {
        super(name, scale, controlled, null,
              MathUtils.normalizeAngle(center, 0),
              MathUtils.normalizeAngle(center, 0) - FastMath.PI,
              MathUtils.normalizeAngle(center, 0) + FastMath.PI);
        this.stephandler  = new Handler();
        this.samplingStep = samplingStep;
        this.earth        = earth;
    }

    public double getAchievedValue() {
        return 0.5 * (minL + maxL);
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
        private static final long serialVersionUID = 7101089708717693731L;

        /** {@inheritDoc} */
        public void reset() {
            // set the initial values at infinite, to make sure they will be updated
            // properly as soon as simulation starts
            minL = Double.POSITIVE_INFINITY;
            maxL = Double.NEGATIVE_INFINITY;
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
                    final Vector3D position = state.getPVCoordinates(earth.getBodyFrame()).getPosition();

                    // convert to latitude/longitude/altitude
                    final GeodeticPoint gp = earth.transform(position, earth.getBodyFrame(), date);
                    final double l = MathUtils.normalizeAngle(gp.getLongitude(), getTargetValue());

                    // update longitude excursion
                    minL = FastMath.min(minL, l);
                    maxL = FastMath.max(maxL, l);

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
