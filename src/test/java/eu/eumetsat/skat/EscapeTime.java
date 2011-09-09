/* Copyright 2011 Eumetsat */

package eu.eumetsat.skat;

import org.apache.commons.math.analysis.UnivariateRealFunction;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.orekit.bodies.BodyShape;
import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.DateDetector;
import org.orekit.propagation.sampling.OrekitFixedStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.errors.OrekitWrapperException;

public class EscapeTime implements UnivariateRealFunction {

    private Propagator propagator;
    private SpacecraftState initialState;
    private BodyShape earth;
    private AbsoluteDate target;

    public EscapeTime(Propagator propagator, BodyShape earth) {
        this.propagator = propagator;
        this.earth      = earth;
    }

    public void setInitialState(SpacecraftState initialState) {
        this.initialState = initialState;
    }

    public void setTargetDate(AbsoluteDate target) {
        this.target = target;
    }

    public double value(double x) throws OrekitWrapperException {
        try {
            return oneManeuverCycle(x, target, null).getDate().durationFrom(initialState.getDate());
        } catch (OrekitException oe) {
            throw new OrekitWrapperException(oe);
        }
    }

    public SpacecraftState oneManeuverCycle(double dV, AbsoluteDate targetDate,
                                            OrekitFixedStepHandler stepHandler) throws OrekitException {
        propagator.resetInitialState(initialState);
        propagator.clearEventsDetectors();
        propagator.addEventDetector(new ImpulseManeuver(new DateDetector(initialState.getDate().shiftedBy(0 * Constants.JULIAN_DAY)),
                                                        new Vector3D(dV, 0, 0), 380.0));
        propagator.addEventDetector(new ImpulseManeuver(new DateDetector(initialState.getDate().shiftedBy(0.5 * Constants.JULIAN_DAY)),
                                                        new Vector3D(dV, 0, 0), 380.0));
        propagator.addEventDetector(new EscapeDetector(100, 1.0, earth, FastMath.toRadians(11.46), FastMath.toRadians(11.6)));

        if (stepHandler != null) {
            propagator.setMasterMode(3600, stepHandler);
        }

        return propagator.propagate(targetDate);

    }

}
