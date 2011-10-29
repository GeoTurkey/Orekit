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

import eu.eumetsat.skat.control.SKControl;

/**
 * Station-keeping control attempting to balance East-West margins
 * throughout a cycle.
 * <p>
 * This control value is:
 * <pre>
 *   (l<sub>E</sub> - max(l(t))) - (min(l(t)) - l<sub>W</sub>)
 * </pre>
 * where l<sub>E</sub> is the East boundary of the longitude slot,
 * l<sub>W</sub> is the West boundary of the longitude slot, l(t) is
 * the spacecraft longitude at time t and the min and max functions
 * are evaluated for the complete cycle duration.
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to zero attempts to have a longitude excursion covered by the satellite
 * during the station-keeping cycle that is well balanced near the center
 * of the longitude slot, the same margin being available on both sides.
 * Setting the target of this control to a non-zero value attempts to have
 * a shifted longitude excursion, the shift being towards West if the target
 * value is positive, and towards East otherwise.
 * </p>
 * @author Luc Maisonobe
 */
public class LongitudeSlotMargins implements SKControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Name of the control law. */
    private final String name;

    /** Scale of the control law. */
    private final double scale;

    /** Longitude margins difference. */
    private final double target;

    /** Longitude slot westward boundary. */
    private final double westBoundary;

    /** Longitude slot eastward boundary. */
    private final double eastBoundary;

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
     * @param westBoundary longitude slot westward boundary
     * @param eastBoundary longitude slot eastward boundary
     * @param target longitude margins difference (should be set to 0.0 if
     * balanced longitude excursion is desired)
     * @param samplingStep step to use for sampling throughout propagation
     * @param earth Earth model to use to compute longitudes
     */
    public LongitudeSlotMargins(final String name, final double scale,
                                final double westBoundary, final double eastBoundary,
                                final double target, final double samplingStep,
                                final BodyShape earth) {
        this.stephandler  = new Handler();
        this.name         = name;
        this.scale        = scale;
        this.target       = target;
        this.westBoundary = westBoundary;
        this.eastBoundary = MathUtils.normalizeAngle(eastBoundary, westBoundary);
        this.samplingStep = samplingStep;
        this.earth        = earth;
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
        return target;
    }

    public double getAchievedValue() {
        return (eastBoundary - maxL) - (minL - westBoundary);
    }

    /** {@inheritDoc} */
    public boolean isConstrained() {
        return true;
    }

    /** {@inheritDoc} */
    public double getMin() {
        return westBoundary - eastBoundary;
    }

    /** {@inheritDoc} */
    public double getMax() {
        return eastBoundary - westBoundary;
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
        private static final long serialVersionUID = 1817488123211175376L;

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

                    // update longitude excursion
                    minL = FastMath.min(minL, gp.getLongitude());
                    maxL = FastMath.max(maxL, gp.getLongitude());

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
