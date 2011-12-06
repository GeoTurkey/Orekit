/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.stat.descriptive.rank.Percentile;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
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
 * Station-keeping control attempting to center East-West longitude
 * excursion around a specified center target throughout a cycle.
 * <p>
 * This control value is:
 * <pre>
 *   max(|l<sub>75</sub> - l<sub>c</sub>|,|l<sub>c</sub> - l<sub>25</sub>|)
 * </pre>
 * where l<sub>75</sub> and l<sub>25</sub> are the spacecraft longitude quantities
 * at 75% and 25% evaluated for the complete cycle duration and l<sub>c</sub> is
 * the center longitude.
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to 0 attempts to have most of the points longitudes covered by the
 * satellite centered around the l<sub>c</sub> longitude during the
 * station-keeping.
 * </p>
 * <p>
 * Using quantiles instead of min/max improves robustness with respect to
 * outliers, which occur when starting far from the desired window for example
 * at the end of LEOP or after a longitude slot change. Here, we ignore 25%
 * outliers on both sides.
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

    /** Target longitude. */
    private final double center;

    /** Longitude sample during station keeping cycle. */
    private List<Double> sample;

    /** Simple constructor.
     * @param name name of the control law
     * @param scalingDivisor divisor to use for scaling the control law
     * @param controlled name of the controlled spacecraft
     * @param lEast longitude slot Eastward boundary
     * @param lWest longitude slot Westward boundary
     * @param samplingStep step to use for sampling throughout propagation
     * @param earth Earth model to use to compute longitudes
     */
    public CenteredLongitude(final String name, final double scalingDivisor,
                             final String controlled,
                             final double lEast, final double lWest,
                             final double samplingStep, final BodyShape earth) {
        super(name, scalingDivisor, controlled, null, 0,
              lWest, MathUtils.normalizeAngle(lEast, lWest));
        this.stephandler  = new Handler();
        this.samplingStep = samplingStep;
        this.earth        = earth;
        this.center       = 0.5 * (getMin() + getMax());
        this.sample       = new ArrayList<Double>();
    }

    /** {@inheritDoc} */
    @Override
    public void initializeRun(final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, AbsoluteDate start, AbsoluteDate end, int rollingCycles)
        throws OrekitException {
        super.initializeRun(maneuvers, propagator, start, end, rollingCycles);
        sample.clear();
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
        final double[] data = new double[sample.size()];
        for (int i = 0; i < data.length; ++i) {
            data[i] = sample.get(i);
        }
        final Percentile p = new Percentile();
        final double l75 = p.evaluate(data, 75.0);
        final double l25 = p.evaluate(data, 25.0);
        return FastMath.max(FastMath.abs(l75 - center), FastMath.abs(center - l25));
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

                    // compute longitude
                    interpolator.setInterpolatedDate(date);
                    final SpacecraftState state = interpolator.getInterpolatedState();
                    final Vector3D position = state.getPVCoordinates(earth.getBodyFrame()).getPosition();
                    final double longitude = FastMath.atan2(position.getY(), position.getX());

                    // check the limits
                    checkLimits(longitude);

                    // add longitude to sample
                    sample.add(MathUtils.normalizeAngle(longitude, getTargetValue()));

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
