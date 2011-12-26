/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.random.RandomGenerator;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Class for introducing randomly missed maneuvers.
 * <p>
 * Missed maneuvers are rescheduled a few orbits after their nominal
 * dates, without being reoptimized.
 * </p>
 * @author Luc Maisonobe
 */
public class MissedManeuver implements ScenarioComponent {

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Name of maneuvers to which this component applies. */
    private final String name;

    /** Miss threshold. */
    private final double missThreshold;

    /** Minimum delay for missed maneuver rescheduling in number of orbits. */
    private final int orbitsSeparation;

    /** Random generator to use for evaluating the error factor. */
    private final RandomGenerator generator;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param name name of maneuvers to which this component applies
     * @param missThreshold miss threshold under which the maneuver is missed
     * (must be a value between 0.0 and 1.0)
     * @param orbitsSeparation minimum delay for missed maneuver rescheduling in number of orbits
     * @param generator random generator to use for evaluating the error factor
     * @exception IllegalArgumentException if miss threshold is not between 0 and 1
     */
    public MissedManeuver(final int[] spacecraftIndices, final String name,
                          final double missThreshold, final int orbitsSeparation,
                          final RandomGenerator generator)
        throws IllegalArgumentException {
        this.spacecraftIndices  = spacecraftIndices.clone();
        this.name               = name;
        if (missThreshold < 0 || missThreshold > 1) {
            throw SkatException.createIllegalArgumentException(SkatMessages.WRONG_MISS_THRESHOLD,
                                                               missThreshold);
        }
        this.missThreshold    = missThreshold;
        this.orbitsSeparation = orbitsSeparation;
        this.generator        = generator;
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
                    // the maneuver can be affected by the error
                    if (generator.nextDouble() < missThreshold) {
                        // the maneuver is missed

                        // remove the original maneuver from the trajectory
                        maneuver.getTrajectory().addManeuver(maneuver.getDate(),
                                                             maneuver.getDeltaV().negate(),
                                                             maneuver.getIsp());

                        // does it need to be replanned ?
                        final ScheduledManeuver m;
                        if (orbitsSeparation > 0.) {
                            // reschedule the missed maneuver
                            final SpacecraftState state = maneuver.getTrajectory().propagate(maneuver.getDate());
                            final double period         = state.getKeplerianPeriod();
                            m = new ScheduledManeuver(maneuver.getModel(), maneuver.getDate().shiftedBy(orbitsSeparation * period),
                                                      maneuver.getDeltaV(),
                                                      maneuver.getThrust(),
                                                      maneuver.getIsp(), maneuver.getTrajectory(),
                                                      true);
                        } else {
                            // the maneuver is really missed
                            m = new ScheduledManeuver(maneuver.getModel(), maneuver.getDate(),
                                                      Vector3D.ZERO,
                                                      maneuver.getThrust(),
                                                      maneuver.getIsp(), maneuver.getTrajectory(),
                                                      true);
                        }

                        m.getTrajectory().addManeuver(m.getDate(), m.getDeltaV(), m.getIsp());
                        modified.add(m);

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
