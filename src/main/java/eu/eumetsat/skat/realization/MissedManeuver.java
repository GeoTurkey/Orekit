/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.random.RandomGenerator;
import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Class for introducing randomly missed maneuvers.
 * <p>
 * Missed maneuvers are simply maneuvers with velocity increment reduced to zero.
 * They still appear in the output but don't change th orbit.
 * </p>
 * @author Luc Maisonobe
 */
public class MissedManeuver implements ScenarioComponent {

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Indicator for applying this error to in-plane maneuvers. */
    private final boolean inPlane;

    /** Indicator for applying this error to out-of-plane maneuvers. */
    private final boolean outOfPlane;

    /** Miss threshold. */
    private final double missThreshold;

    /** Random generator to use for evaluating the error factor. */
    private final RandomGenerator generator;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param inPlane if true, the error applies to in-plane maneuvers
     * @param outOfPlane if true, the error applies to out-of-plane maneuvers
     * @param missThreshold miss threshold under which the maneuver is missed
     * (must be a value between 0.0 and 1.0)
     * @param generator random generator to use for evaluating the error factor
     * @exception IllegalArgumentException if miss threshold is not between 0 and 1
     */
    public MissedManeuver(final int[] spacecraftIndices,
                          final boolean inPlane, final boolean outOfPlane,
                          final double missThreshold,
                          final RandomGenerator generator)
        throws IllegalArgumentException {
        this.spacecraftIndices  = spacecraftIndices.clone();
        this.inPlane            = inPlane;
        this.outOfPlane         = outOfPlane;
        if (missThreshold < 0 || missThreshold > 1) {
            throw SkatException.createIllegalArgumentException(SkatMessages.WRONG_MISS_THRESHOLD,
                                                               missThreshold);
        }
        this.missThreshold      = missThreshold;
        this.generator          = generator;
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
                if ((inPlane && maneuver.isInPlane()) || (outOfPlane && !(maneuver.isInPlane()))) {
                    // the maneuver is affected by the error
                    if (generator.nextDouble() < missThreshold) {
                        // the maneuver is missed
                        modified.add(new ScheduledManeuver(maneuver.getName(), maneuver.isInPlane(),
                                                           maneuver.getDate(), Vector3D.ZERO,
                                                           maneuver.getThrust(), maneuver.getIsp(),
                                                           maneuver.getTrajectory()));
                    } else {
                        // the maneuver is realized as scheduled
                        modified.add(maneuver);
                    }
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
