package eu.eumetsat.skat.simulation;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.DateDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.ControlParameter;
import eu.eumetsat.skat.control.ControlParametersSet;

/**
 * Class for maneuver simulation.
 * <p>
 * This class performs an impulse maneuver with tunable date and
 * velocity increment
 * </p>
 * @author Luc Maisonobe
 */
public class TunableManeuver implements ControlParametersSet, EventDetector {

    /** Serializable UID. */
    private static final long serialVersionUID = 7193368247300270766L;

    /** Thrust direction in spacecraft frame. */
    private final Vector3D direction;

    /** Specific impulse. */
    private double isp;

    /** Tunable velocity increment. */
    private final ControlParameter velocityIncrement;

    /** Reference date of the maneuver. */
    private final AbsoluteDate reference;

    /** Tunable date offset. */
    private final ControlParameter dateOffset;

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

    /** {@inheritDoc} */
    public double g(SpacecraftState s) throws OrekitException {
        resetIfInvalid();
        return current.g(s);
    }

    /** {@inheritDoc} */
    public int eventOccurred(SpacecraftState s, boolean increasing) throws OrekitException {
        resetIfInvalid();
        return current.eventOccurred(s, increasing);
    }

    /** {@inheritDoc} */
    public SpacecraftState resetState(SpacecraftState oldState) throws OrekitException {
        resetIfInvalid();
        return current.resetState(oldState);
    }

    /** {@inheritDoc} */
    public double getThreshold() {
        resetIfInvalid();
        return current.getThreshold();
    }

    /** {@inheritDoc} */
    public double getMaxCheckInterval() {
        resetIfInvalid();
        return current.getMaxCheckInterval();
    }

    /** {@inheritDoc} */
    public int getMaxIterationCount() {
        resetIfInvalid();
        return current.getMaxIterationCount();
    }

    /** Reset the current impulse maneuver if it has been invalidated.
     */
    private void resetIfInvalid() {
        if (current == null) {
            AbsoluteDate triggerDate = reference.shiftedBy(dateOffset.getValue());
            current = new ImpulseManeuver(new DateDetector(triggerDate),
                                          new Vector3D(velocityIncrement.getValue(),
                                                       direction),
                                          isp);
        }
    }

    /** Local class for control parameters invalidating maneuver at parameter changes. */
    private class ManeuverParameter extends ControlParameter {

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

    }

    /** {@inheritDoc} */
    public List<ControlParameter> getParameters() {
        final List<ControlParameter> list = new ArrayList<ControlParameter>(2);
        list.add(dateOffset);
        list.add(velocityIncrement);
        return list;
    }

}
