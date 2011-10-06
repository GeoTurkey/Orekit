/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.events.DateDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.SKParameter;

/**
 * Class for maneuver simulation.
 * <p>
 * This class performs an impulse maneuver with tunable date and
 * velocity increment
 * </p>
 * @author Luc Maisonobe
 */
public class TunableManeuver {

    /** Thrust direction in spacecraft frame. */
    private final Vector3D direction;

    /** Specific impulse. */
    private double isp;

    /** Tunable velocity increment. */
    private final SKParameter velocityIncrement;

    /** Reference date of the maneuver. */
    private final AbsoluteDate reference;

    /** Tunable date offset. */
    private final SKParameter dateOffset;

    /** Current impulse maneuver (with already set parameters). */
    private ImpulseManeuver current;

    /** Simple constructor.
     * @param name name of the maneuver
     * @param direction thrust direction in spacecraft frame
     * @param isp engine specific impulse (s)
     * @param minIncrement minimal allowed value for velocity increment
     * @param maxIncrement maximal allowed value for velocity increment
     * @param reference maneuver reference date
     * @param minDateOffset offset for earliest allowed maneuver date
     * @param maxDateOffset offset for latest allowed maneuver date
     */
    public TunableManeuver(final String name,
                           final Vector3D direction, final double isp,
                           final double minIncrement, final double maxIncrement,
                           final AbsoluteDate reference,
                           final double minDateOffset, final double maxDateOffset) {
        this.direction    = direction.normalize();
        this.isp          = isp;
        this.reference    = reference;
        dateOffset        = new ManeuverParameter(name + " (date)",
                                                  minDateOffset, maxDateOffset,
                                                  0.5 * (minDateOffset + maxDateOffset),
                                                  true);
        velocityIncrement = new ManeuverParameter(name + " (dV)",
                                                  minIncrement, maxIncrement,
                                                  0.5 * (minIncrement + maxIncrement),
                                                  true);
        current           = null;
    }

    /** Local class for control parameters invalidating maneuver at parameter changes. */
    private class ManeuverParameter extends SKParameter {

        /** Simple constructor.
         * @param name name of the parameter
         * @param min minimal allowed value for the parameter
         * @param max maximal allowed value for the parameter
         * @param value current value of the parameter
         * @param tunable tunable flag
         */
        public ManeuverParameter(final String name,
                                 final double min, final double max,
                                 final double value, final boolean tunable) {
            super(name, min, max, value, tunable);
        }

        /** {@inheritDoc} */
        protected void valueChanged() {
            current = null;
        }

        /** {@inheritDoc} */
        public EventDetector getEventDetector() {
            return getManeuver();
        }

        /** {@inheritDoc} */
        public OrekitStepHandler getStepHandler() {
            return null;
        }

    }

    /** Get the maneuver parameters.
     * @return list of maneuver parameters
     */
    public List<SKParameter> getParameters() {
        final List<SKParameter> list = new ArrayList<SKParameter>(2);
        list.add(dateOffset);
        list.add(velocityIncrement);
        return list;
    }

    /** Get the maneuver corresponding to the current value of the parameters.
     * @return maneuver corresponding to the current value of the parameters
     */
    public ImpulseManeuver getManeuver() {

        if (current == null) {
            // the parameters value have changed, thus invalidating the maneuver,
            // build a new valid one
            AbsoluteDate triggerDate = reference.shiftedBy(dateOffset.getValue());
            current = new ImpulseManeuver(new DateDetector(triggerDate),
                                          new Vector3D(velocityIncrement.getValue(),
                                                       direction),
                                          isp);
        }

        return current;

    }

}
