/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.stat.descriptive.rank.Percentile;
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
 * excursion around a specified center target throughout a cycle.
 * <p>
 * This control value is:
 * <pre>
 *   max(|l<sub>75</sub> - l<sub>c</sub>|,|l<sub>c</sub> - l<sub>25</sub>|)
 * </pre>
 * where l<sub>75</sub> and l<sub>25</sub> are the spacecraft longitude quantiles
 * evaluated for the complete cycle duration and l<sub>c</sub> is the center longitude.
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to 0 attempts to have a longitude excursion covered by the
 * satellite centered around the l<sub>c</sub> longitude during the
 * station-keeping.
 * </p>
 * <p>
 * Using percentiles instead of min/max improves robustness with respect to
 * outliers, which occur when starting far from the desired window.
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
        super(name, scale, controlled, null, 0, -FastMath.PI, FastMath.PI);
        this.stephandler  = new Handler();
        this.samplingStep = samplingStep;
        this.earth        = earth;
        this.center       = center;
        this.sample       = new ArrayList<Double>();
    }

    /** {@inheritDoc} */
    @Override
    public void initializeRun() {
        sample.clear();
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
        final double[] data = new double[sample.size()];
        for (int i = 0; i < data.length; ++i) {
            data[i] = sample.get(i);
        }
        final Percentile p = new Percentile();
        final double up  = p.evaluate(data, 75.0);
        final double low = p.evaluate(data, 25.0);
        return FastMath.max(FastMath.abs(up - center), FastMath.abs(center - low));
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

                    // add longitude to sample
                    sample.add(MathUtils.normalizeAngle(l, getTargetValue()));

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
