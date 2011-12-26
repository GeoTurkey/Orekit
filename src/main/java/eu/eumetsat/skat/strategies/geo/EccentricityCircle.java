/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.util.FastMath;
import org.orekit.bodies.CelestialBody;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.SkatException;

/**
 * Station-keeping control attempting to follow perigee solar pointing
 * with respect to a specified eccentricity circle.
 * <p>
 * This control value is:
 * <pre>
 *   median(&delta;(t))
 * </pre>
 * where &delta;(t) = &radic; [(e<sub>x</sub> - p<sub>x</sub>)<sup>2</sup> +
 * (e<sub>y</sub> - p<sub>y</sub>)<sup>2</sup>] is the distance between the
 * current eccentricity vector (e<sub>x</sub>, e<sub>y</sub>) and a solar
 * pointing eccentricity vector for eccentricity circle centered at
 * (c<sub>x</sub>, c<sub>y</sub>) and radius r.
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to 0 attempts to have the eccentricity vector motion as close as
 * possible to the perfect solar pointing (p<sub>x</sub>, p<sub>y</sub>)
 * for the specified circle.
 * </p>
 * <p>
 * Using a median instead of a mean improves robustness with respect to
 * outliers, which occur when starting far from the desired window.
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

    /** Circle radius. */
    private final double radius;

    /** Sun model. */
    private CelestialBody sun;

    /** Sample of eccentricity offset during station keeping cycle. */
    private List<Double> sample;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param centerX abscissa of the circle center
     * @param centerY ordinate of the circle center
     * @param radius radius of the circle
     * @param sun Sun model
     * @param samplingStep step to use for sampling throughout propagation
     */
    public EccentricityCircle(final String name, final String controlledName, final int controlledIndex,
                              final double centerX, final double centerY, final double radius,
                              final CelestialBody sun, final double samplingStep) {
        super(name, controlledName, controlledIndex, null, -1, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        this.stephandler  = new Handler();
        this.centerX      = centerX;
        this.centerY      = centerY;
        this.radius       = radius;
        this.sun          = sun;
        this.samplingStep = samplingStep;
        this.sample       = new ArrayList<Double>();
    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException {
    }

    /** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException {
        // TODO
        throw SkatException.createInternalError(null);
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
        public void init(final SpacecraftState s0, final AbsoluteDate t) {
            resetMarginsChecks();
            sample.clear();
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

                    // compute current eccentricity
                    interpolator.setInterpolatedDate(date);
                    final SpacecraftState state  = interpolator.getInterpolatedState();
                    final EquinoctialOrbit orbit = (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state.getOrbit());

                    // compute perfect solar pointing
                    final double alphaSun = sun.getPVCoordinates(date, state.getFrame()).getPosition().getAlpha();
                    final double pX       = centerX + radius * FastMath.cos(alphaSun);
                    final double pY       = centerY + radius * FastMath.sin(alphaSun);

                    // add eccentricity offset to sample
                    sample.add(FastMath.hypot(orbit.getEquinoctialEx() - pX,
                                              orbit.getEquinoctialEy() - pY));

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
