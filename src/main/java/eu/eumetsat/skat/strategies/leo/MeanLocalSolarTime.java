/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.stat.descriptive.rank.Percentile;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.AbstractDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;

/**
 * Station-keeping control attempting to get mean local solar time in a deadband.
 * The deadband is located around a main value {@link #center}. A tolerance margin
 * can be define through the minSolarTime and maxSolarTime parameters, defined by
 * construction. If a violation occurs by getting out of the [minSolarTime, maxSolarTime] 
 * interval, a notifier will be triggered and this information will be monitored.
 *  <p>
 * This control value is:
 * <pre> max(|MLST<sub>75</sub> - MLST<sub>c</sub>|,|MLST<sub>c</sub> - MLST<sub>25</sub>|)</pre>
 * where MLST<sub>75</sub> and MLST<sub>25</sub> are the spacecraft mean local solar time 1st and 3rd
 * quartiles evaluated for the complete cycle duration and MLST<sub>c</sub> is the target mean local
 * solar time.
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to MLST<sub>c</sub> attempts to have most of the points mean local solar time
 * for the satellite centered around the target mean local solar time during the
 * station-keeping. 
 * </p>
 * <p>
 * Using quantiles instead of min/max improves robustness with respect to
 * outliers, which occur when starting far from the desired window. Here, we ignore 25%
 * outliers on both sides.
 * </p>
 */
public class MeanLocalSolarTime extends AbstractSKControl {

    /** Associated event detector. */
    private final EventDetector eventDetector;

    /** Sampled offset to target solar time. */
    private final List<Double> sample;
    
    /** Solar time target */
    private final double center;

    /** Simple constructor.
     * @param name name of the control law
     * @param scalingDivisor divisor to use for scaling the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param earth Earth model
     * @param latitude latitude at which solar time should be computed
     * @param ascending if true, solar time is computed when crossing the
     * specified latitude from south to north
     * @param solarTime target solar time ((in fractional hour, i.e 9h30 = 9.5)
     * @param minSolarTime minimum accepted solar time (in fractional hour)
     * @param maxSolarTime maximum accepted solar time (in fractional hour)
     * @exception OrekitException if the UTC-TAI correction cannot be loaded
     */
    public MeanLocalSolarTime(final String name, final double scalingDivisor,
                              final String controlledName, final int controlledIndex,
                              final BodyShape earth,
                              final double latitude,
                              final boolean ascending,
                              final double solarTime,
                              final double minSolarTime,
                              final double maxSolarTime)
        throws OrekitException {
        super(name, scalingDivisor, controlledName, controlledIndex, null, -1,
              0., minSolarTime, maxSolarTime);
        this.eventDetector = new Detector(600.0, 1.0e-3, earth, latitude, ascending);
        this.sample = new ArrayList<Double>();
        this.center = solarTime;
    }

    /** {@inheritDoc} */
    @Override
    public void initializeRun(final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, AbsoluteDate start, AbsoluteDate end, int rollingCycles)
        throws OrekitException {
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
        return eventDetector;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return null;
    }

    /** Inner class for events handling. */
    private class Detector extends AbstractDetector {

        /** Serializable UID. */
        private static final long serialVersionUID = -5023948730619118388L;

        /** Earth model. */
        private final BodyShape earth;

        /** Latitude at which solar time should be computed. */
        private final double latitude;

        /** Indicator for solar time computation direction. */
        private final boolean ascending;

        /** GMOD Frame. */
        private final GMODFrame gmod;

        /** UTC time scale. */
        private final TimeScale utc;


        /** Simple constructor
         * @param maxCheck maximum checking interval (s)
         * @param threshold convergence threshold (s)
         * @param earth Earth model
         * @param latitude latitude at which solar time should be computed
         * @param ascending if true, solar time is computed when crossing the
         * @param targetSolarTime target solar time
         * @exception OrekitException if the UTC-TAI correction cannot be loaded
         */
        public Detector(final double maxCheck, final double threshold,
                        final BodyShape earth, final double latitude,
                        final boolean ascending)
            throws OrekitException {
            super(maxCheck, threshold);
            this.earth           = earth;
            this.latitude        = latitude;
            this.ascending       = ascending;
            this.gmod            = new GMODFrame();
            this.utc             = TimeScalesFactory.getUTC();
        }

        /** {@inheritDoc} */
        public void init(final SpacecraftState s0, final AbsoluteDate t) {
            resetLimitsChecks();
            sample.clear();
        }

        /** {@inheritDoc} */
        public double g(final SpacecraftState s)
            throws OrekitException {
            final Vector3D position = s.getPVCoordinates(earth.getBodyFrame()).getPosition();
            final GeodeticPoint gp  = earth.transform(position, earth.getBodyFrame(), s.getDate());
            return gp.getLatitude() - latitude;
        }

        /** {@inheritDoc} */
        public Action eventOccurred(final SpacecraftState s, final boolean increasing)
            throws OrekitException {

            if (increasing == ascending) {
                // we crossed the specified latitude in the expected direction

                // compute angle between Sun and spacecraft in the equatorial plane
                final Frame gcrf =  FramesFactory.getGCRF();
                final Vector3D spacecraftPos = s.getPVCoordinates(gcrf).getPosition();
                final double time = s.getDate().getComponents(utc).getTime().getSecondsInDay();
                final double sunAlpha = gmod.getMeanSiderealTime(s.getDate()) +
                                        (time / Constants.JULIAN_DAY - 0.5) * 2.0 * FastMath.PI;
                final double dAlpha = MathUtils.normalizeAngle(spacecraftPos.getAlpha() - sunAlpha, 0);

                // convert the angle to solar time
                final double achievedSolarTime = 12.0 * (1.0 + dAlpha / FastMath.PI);

                checkLimits(achievedSolarTime);
                sample.add(achievedSolarTime);
            }
            // just continue propagation
            return Action.CONTINUE;
        }
    }
}
