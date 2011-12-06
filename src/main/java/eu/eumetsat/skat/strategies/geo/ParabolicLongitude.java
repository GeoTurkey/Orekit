/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.optimization.fitting.PolynomialFitter;
import org.apache.commons.math.optimization.general.LevenbergMarquardtOptimizer;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.frames.Transform;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;

/**
 * Station-keeping control attempting to center et parabolique longitude
 * excursion around a specified center target throughout a cycle.
 * <p>
 * This control value is:
 * <pre>
 *   &sum;(l(t)-l<sub>p</sub>(y))<sup>2</sup>
 * </pre>
 * where l(t) is the spacecraft longitude and l<sub>p</sub>(t) is a centered
 * parabolic longitude motion with the same acceleration as the observed motion.
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to 0 attempts to have most of the points longitudes covered by the
 * satellite centered around the l<sub>c</sub> longitude during the
 * station-keeping.
 * </p>
 * @author Luc Maisonobe
 */
public class ParabolicLongitude extends AbstractSKControl {

    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Duration of the ignored start part of the cycle. */
    private final double ignoredStartDuration;

    /** Earth model to use to compute longitudes. */
    private final BodyShape earth;

    /** Target longitude. */
    private final double center;

    /** Cycle start. */
    private AbsoluteDate start;

    /** First maneuver date. */
    private AbsoluteDate firstManeuver;

    /** Cycle duration. */
    private double cycleDuration;

    /** Date sample during station keeping cycle. */
    private List<Double> dateSample;

    /** Longitude sample during station keeping cycle. */
    private List<Double> longitudeSample;

    /** Simple constructor.
     * @param name name of the control law
     * @param scalingDivisor divisor to use for scaling the control law
     * @param controlled name of the controlled spacecraft
     * @param ignoredStartDuration duration of the ignored start part of the cycle
     * @param lEast longitude slot Eastward boundary
     * @param lWest longitude slot Westward boundary
     * @param samplingStep step to use for sampling throughout propagation
     * @param earth Earth model to use to compute longitudes
     */
    public ParabolicLongitude(final String name, final double scalingDivisor,
                              final String controlled, final double ignoredStartDuration,
                              final double lEast, final double lWest,
                              final double samplingStep, final BodyShape earth) {
        super(name, scalingDivisor, controlled, null, 0,
              lWest, MathUtils.normalizeAngle(lEast, lWest));
        this.stephandler          = new Handler();
        this.samplingStep         = samplingStep;
        this.ignoredStartDuration = ignoredStartDuration;
        this.earth                = earth;
        this.center               = 0.5 * (getMin() + getMax());
        this.dateSample           = new ArrayList<Double>();
        this.longitudeSample      = new ArrayList<Double>();
    }

    /** {@inheritDoc} */
    @Override
    public void initializeRun(final ScheduledManeuver[] maneuvers,
                              final Propagator propagator,
                              final AbsoluteDate start, final AbsoluteDate end,
                              final int rollingCycles)
        throws OrekitException {
        super.initializeRun(maneuvers, propagator, start, end, rollingCycles);
        this.start         = start;
        this.firstManeuver = maneuvers[0].getDate();
        this.cycleDuration = end.durationFrom(start) / rollingCycles;
        dateSample.clear();
        longitudeSample.clear();
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {

        // fit the longitude motion as a parabola
        final PolynomialFitter fitter = new PolynomialFitter(2, new LevenbergMarquardtOptimizer());
        for (int i = 0; i < dateSample.size(); ++i) {
            final double date = dateSample.get(i);
            final double dateInCycle = date - cycleDuration * FastMath.floor(date / cycleDuration);
            fitter.addObservedPoint(dateInCycle, longitudeSample.get(i));
        }
        double[] coefficients = fitter.fit();

        // change the polynomials coefficients so the parabola performed
        // from start to end of cycle is centered around center longitude
        // and its vertex is reached at cycle center date
        coefficients[0] = center + coefficients[2] * cycleDuration * cycleDuration / 8;
        coefficients[1] = - coefficients[2] * cycleDuration;
        PolynomialFunction reference = new PolynomialFunction(coefficients);

        // compute residuals with respect to reference parabola
        double sum2 = 0;
        for (int i = 0; i < dateSample.size(); ++i) {
            final double date = dateSample.get(i);
            final double dateInCycle = date - cycleDuration * FastMath.floor(date / cycleDuration);
            final double residual = longitudeSample.get(i) - reference.value(dateInCycle);
            sum2 += residual * residual;
        }

        return FastMath.sqrt(sum2) / dateSample.size();

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
        public void init(final SpacecraftState s0, final AbsoluteDate t) {
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

                    if (date.durationFrom(start) > ignoredStartDuration) {
                        // compute mean longitude argument
                        interpolator.setInterpolatedDate(date);
                        final SpacecraftState state = interpolator.getInterpolatedState();
                        final EquinoctialOrbit orbit = (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state.getOrbit());
                        final double lambdaV = orbit.getLv();
                        final double lambdaM = orbit.getLM();

                        // compute sidereal time
                        final Transform transform = earth.getBodyFrame().getTransformTo(state.getFrame(), date);
                        final double theta = transform.transformVector(Vector3D.PLUS_I).getAlpha();

                        // compute mean longitude
                        final double trueLongitude = MathUtils.normalizeAngle(lambdaV - theta, center);
                        final double meanLongitude = MathUtils.normalizeAngle(lambdaM - theta, center);

                        // check the limits
                        checkLimits(trueLongitude);

                        // add point to sample
                        dateSample.add(date.durationFrom(firstManeuver));
                        longitudeSample.add(meanLongitude);
                    }

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
