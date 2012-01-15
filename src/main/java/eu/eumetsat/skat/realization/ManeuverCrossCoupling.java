/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Rotation;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Class for computing cross-coupling in maneuvers.
 * <p>
 * Cross-coupling is due to deflection of the exhaust from
 * thrusters by spacecraft parts (solar arrays, antennas ...).
 * It is modeled by a small rotation of the thrust direction.
 * </p>
 * <p>
 * From an engineering point of view, cross-coupling is often
 * defined as a percentage or ratio with respect to the original
 * thrust. So the user has to define a nominal direction for the
 * thrust in spacecraft frame (say Y axis for example), a coupling
 * direction in spacecraft frame (say X axis for example) and a
 * coupling ratio r. From these defining elements, the coupling
 * model will be a rotation around axis -Z with angle asin(r).
 * </p>
 * @author Luc Maisonobe
 */
public class ManeuverCrossCoupling implements ScenarioComponent {

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Name of maneuvers to which this component applies. */
    private final String name;

    /** Coupling rotation. */
    private final Rotation coupling;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param name name of maneuvers to which this component applies
     * @param nominalDirection nominal direction of the thrust in spacecraft frame
     * @param couplingDirection coupling direction in spacecraft frame
     * @param couplingRatio ratio of the coupling along coupling axis
     * (must be between -1 and 1)
     * @exception IllegalArgumentException if coupling ratio is not between -1 and 1
     * or if coupling direction is aligned with nominal thrust direction
     */
    public ManeuverCrossCoupling(final int[] spacecraftIndices, final String name,
                                 final Vector3D nominalDirection, final Vector3D couplingDirection,
                                 final double couplingRatio)
        throws IllegalArgumentException {
        this.spacecraftIndices  = spacecraftIndices.clone();
        this.name               = name;
        if (couplingRatio < -1 || couplingRatio > 1) {
            throw SkatException.createIllegalArgumentException(SkatMessages.WRONG_COUPLING,
                                                               couplingRatio);
        }
        if (Vector3D.angle(couplingDirection, nominalDirection) < 0.1) {
            throw SkatException.createIllegalArgumentException(SkatMessages.ALIGNED_COUPLING_AXES,
                                                               couplingDirection.getX(),
                                                               couplingDirection.getY(),
                                                               couplingDirection.getZ());
        }
        this.coupling = new Rotation(Vector3D.crossProduct(nominalDirection, couplingDirection),
                                     FastMath.asin(couplingRatio));
    }

    /** {@inheritDoc} */
    public void setCycleEnd(final AbsoluteDate cycleEnd) {
        // nothing to do here
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException, SkatException {

        ScenarioState[] updated = originals.clone();

        for (int i = 0; i < spacecraftIndices.length; ++i) {

            // select the current spacecraft affected by this component
            final int index = spacecraftIndices[i];
            final List<ScheduledManeuver> rawManeuvers = originals[index].getManeuvers();
            if (rawManeuvers == null) {
                throw new SkatException(SkatMessages.NO_MANEUVERS_STATE,
                                        originals[index].getName(), originals[index].getCyclesNumber());
            }

            // prepare a list for holding the modified maneuvers
            List<ScheduledManeuver> modified = new ArrayList<ScheduledManeuver>();

            // modify the maneuvers
            for (final ScheduledManeuver maneuver : rawManeuvers) {
                if (maneuver.getName().equals(name)) {
                    // the maneuver is affected by the coupling
                    final ScheduledManeuver m = new ScheduledManeuver(maneuver.getModel(), maneuver.getDate(),
                                                                      coupling.applyTo(maneuver.getDeltaV()),
                                                                      maneuver.getThrust(),
                                                                      maneuver.getIsp(), maneuver.getTrajectory(),
                                                                      false);
                    maneuver.getTrajectory().addEffect(new SmallManeuverAnalyticalModel(maneuver.getStateBefore(),
                                                                                        maneuver.getDeltaV().negate(),
                                                                                        maneuver.getIsp()));
                    m.getTrajectory().addEffect(new SmallManeuverAnalyticalModel(m.getStateBefore(),
                                                                                 m.getDeltaV(),
                                                                                 m.getIsp()));
                    modified.add(m);
                } else {
                    // the maneuver is immune to the error
                    modified.add(maneuver);
                }
            }

            // update the state
            updated[index] = originals[index].updateManeuvers(modified);

        }

        // return an updated states
        return updated;

    }

}
