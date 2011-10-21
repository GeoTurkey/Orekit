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
 * This class represents an impulse maneuver with free parameters for date
 * and velocity increment.
 * @author Luc Maisonobe
 */
public class TunableManeuver {

    /** Indicator for in-plane maneuvers. */
    private final boolean inPlane;

    /** Thrust direction in spacecraft frame. */
    private final Vector3D direction;

    /** Specific impulse. */
    private final double isp;

    /** Tunable velocity increment. */
    private final SKParameter velocityIncrement;

    /** Nominal offset with respect to reference date. */
    private final double nominal;

    /** Reference date of the maneuver. */
    private AbsoluteDate reference;

    /** Tunable date offset. */
    private final SKParameter dateOffset;

    /** Current impulse maneuver (with already set parameters). */
    private ImpulseManeuver current;

    /** Simple constructor.
     * @param name name of the maneuver
     * @param inPlane if true, the maneuver is considered to be in-plane
     * @param direction thrust direction in spacecraft frame
     * @param isp engine specific impulse (s)
     * @param minIncrement minimal allowed value for velocity increment
     * @param maxIncrement maximal allowed value for velocity increment
     * @param nominal nominal offset with respect to reference date
     * @param minDateOffset offset for earliest allowed maneuver date
     * @param maxDateOffset offset for latest allowed maneuver date
     */
    public TunableManeuver(final String name, final boolean inPlane,
                           final Vector3D direction, final double isp,
                           final double minIncrement, final double maxIncrement,
                           final double nominal,
                           final double minDateOffset, final double maxDateOffset) {
        this.inPlane      = inPlane;
        this.direction    = direction.normalize();
        this.isp          = isp;
        this.nominal      = nominal;
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

    /** Set the reference date.
     * @param reference reference date
     */
    public void setReferenceDate(final AbsoluteDate reference) {
        this.reference = reference;
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

            if (current == null) {
                // the parameters value have changed, thus invalidating the maneuver,
                // build a new valid one
                AbsoluteDate triggerDate = reference.shiftedBy(nominal + dateOffset.getValue());
                current = new ImpulseManeuver(new DateDetector(triggerDate),
                                              new Vector3D(velocityIncrement.getValue(), direction),
                                              isp);
            }

            return current;

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
    public ScheduledManeuver getManeuver() {
        return new ScheduledManeuver(inPlane,
                                     reference.shiftedBy(nominal + dateOffset.getValue()),
                                     new Vector3D(velocityIncrement.getValue(), direction),
                                     isp);
    }

}
