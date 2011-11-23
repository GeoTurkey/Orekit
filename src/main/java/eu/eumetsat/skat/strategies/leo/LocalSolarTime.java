/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.stat.descriptive.rank.Median;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.AbstractDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;

/**
 * Station-keeping control attempting to get local solar time in a deadband.
 */
public class LocalSolarTime extends AbstractSKControl {

    /** Associated event detector. */
    private final EventDetector eventDetector;

    /** Sampled offset to target solar time. */
    private final List<Double> sample;

    /** Simple constructor.
     * @param name name of the control law
     * @param scalingDivisor divisor to use for scaling the control law
     * @param controlled name of the controlled spacecraft
     * @param earth Earth model
     * @param sun Sun model
     * @param latitude latitude at which solar time should be computed
     * @param ascending if true, solar time is computed when crossing the
     * specified latitude from south to north
     * @param solarTime target solar time
     */
    public LocalSolarTime(final String name, final double scalingDivisor,
                          final String controlled,
                          final BodyShape earth, final CelestialBody sun,
                          final double latitude, boolean ascending,
                          final double solarTime) {
        super(name, scalingDivisor, controlled, null, solarTime, 0.0, 24.0);
        this.eventDetector =
                new Detector(600.0, 1.0e-3, earth, sun, latitude, ascending, solarTime);
        this.sample = new ArrayList<Double>();
    }

    /** {@inheritDoc} */
    @Override
    public void initializeRun(final ScheduledManeuver[] maneuvers,
                              final Propagator propagator) {
        sample.clear();
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
        final double[] data = new double[sample.size()];
        for (int i = 0; i < data.length; ++i) {
            data[i] = sample.get(i);
        }
        return new Median().evaluate(data);
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

        /** Sun model. */
        private final CelestialBody sun;

        /** Latitude at which solar time should be computed. */
        private final double latitude;

        /** Indicator for solar time computation direction. */
        private final boolean ascending;

        /** Target solar time. */
        private final double targetSolarTime;

        /** Simple constructor
         * @param maxCheck maximum checking interval (s)
         * @param threshold convergence threshold (s)
         * @param earth Earth model
         * @param sun Sun model
         * @param latitude latitude at which solar time should be computed
         * @param ascending if true, solar time is computed when crossing the
         * @param targetSolarTime target solar time
         */
        public Detector(final double maxCheck, final double threshold,
                        final BodyShape earth, final CelestialBody sun,
                        final double latitude, final boolean ascending, final double targetSolarTime) {
            super(maxCheck, threshold);
            this.earth           = earth;
            this.sun             = sun;
            this.latitude        = latitude;
            this.ascending       = ascending;
            this.targetSolarTime = targetSolarTime;
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
                final Vector3D sunPos = sun.getPVCoordinates(s.getDate(), gcrf).getPosition();
                final double dAlpha = MathUtils.normalizeAngle(spacecraftPos.getAlpha() - sunPos.getAlpha(), 0);

                // convert the angle to solar time
                final double achievedSolarTime = 12.0 * (1.0 + dAlpha / FastMath.PI);

                sample.add(achievedSolarTime - targetSolarTime);

            }

            // just continue propagation
            return Action.CONTINUE;

        }

    }

}
