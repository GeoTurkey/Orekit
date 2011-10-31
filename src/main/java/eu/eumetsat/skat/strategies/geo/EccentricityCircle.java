/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import org.apache.commons.math.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.AbstractSKControl;

/**
 * Station-keeping control attempting to follow a specified eccentricity circle.
 * <p>
 * This control value is:
 * <pre>
 *   (max(&delta;(t)) + min(&delta;(t))) / 2
 * </pre>
 * where &delta;(t) = &radic; [(e<sub>x</sub> - c<sub>x</sub>)<sup>2</sup> +
 * (e<sub>y</sub> - c<sub>y</sub>)<sup>2</sup>] is the distance between the
 * current eccentricity vector (e<sub>x</sub>, e<sub>y</sub>) and a reference
 * point at (c<sub>x</sub>, c<sub>y</sub>).
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to r attempts to have the eccentricity vector motion as close as
 * possible to the circle centered at (c<sub>x</sub>, c<sub>y</sub>) and with
 * radius r.
 * </p>
 * @author Luc Maisonobe
 */
public class EccentricityCircle extends AbstractSKControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Abscissa of the circle center. */
    private final double centerX;

    /** Ordinate of the circle center. */
    private final double centerY;

    /** Minimal delta reached during station keeping cycle. */
    private double minDelta;

    /** Maximal delta reached during station keeping cycle. */
    private double maxDelta;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Simple constructor.
     * @param name name of the control law
     * @param scale of the control law
     * @param controlled name of the controlled spacecraft
     * @param centerX abscissa of the circle center
     * @param centerY ordinate of the circle center
     * @param radius radius of the circle
     * @param samplingStep step to use for sampling throughout propagation
     */
    public EccentricityCircle(final String name, final double scale, final String controlled,
                              final double centerX, final double centerY, final double radius,
                              final double samplingStep) {
        super(name, scale, controlled, null, radius, -1.0, 1.0);
        this.stephandler  = new Handler();
        this.centerX      = centerX;
        this.centerY      = centerY;
        this.samplingStep = samplingStep;
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
        return 0.5 * (minDelta + maxDelta);
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
        private static final long serialVersionUID = 220407581859026265L;

        /** {@inheritDoc} */
        public void reset() {
            // set the initial values at infinite, to make sure they will be updated
            // properly as soon as simulation starts
            minDelta = Double.POSITIVE_INFINITY;
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
                for (AbsoluteDate date = minDate; date.compareTo(maxDate) < 0; date = date.shiftedBy(samplingStep)) {

                    // compute distance to circle center
                    final SpacecraftState state  = interpolator.getInterpolatedState();
                    final EquinoctialOrbit orbit = (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state.getOrbit());
                    final double delta           = FastMath.hypot(orbit.getEquinoctialEx() - centerX,
                                                                  orbit.getEquinoctialEy() - centerY);

                    // update eccentricity excursion
                    minDelta = FastMath.min(minDelta, delta);
                    maxDelta = FastMath.max(maxDelta, delta);

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
