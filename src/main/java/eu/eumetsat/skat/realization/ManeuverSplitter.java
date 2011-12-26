/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Class for splitting large maneuvers as several smaller ones.
 * @author Luc Maisonobe
 */
public class ManeuverSplitter implements ScenarioComponent {

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Name of maneuvers to which this component applies. */
    private final String name;

    /** Maximum allowed velocity increment. */
    private final double maxDV;

    /** Minimum time between split parts in number of orbits. */
    private final int orbitsSeparation;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param name name of maneuvers to which this component applies
     * @param maxDV maximum allowed velocity increment
     * @param orbitsSeparation minimum time between split parts in number of orbits
     */
    public ManeuverSplitter(final int[] spacecraftIndices, final String name,
                            final double maxDV, final int orbitsSeparation)
        throws IllegalArgumentException {
        this.spacecraftIndices = spacecraftIndices.clone();
        this.name              = name;
        this.maxDV             = maxDV;
        this.orbitsSeparation  = orbitsSeparation;
        if (orbitsSeparation <= 0) {
            throw SkatException.createIllegalArgumentException(SkatMessages.WRONG_SPLIT_DT,
                                                               orbitsSeparation);
        }
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
                if (maneuver.getName().equals(name) && (maneuver.getDeltaV().getNorm() > maxDV)) {

                    // the maneuver should be split
                    final SpacecraftState state = maneuver.getTrajectory().propagate(maneuver.getDate());
                    final double period         = state.getKeplerianPeriod();
                    final int nbParts           = (int) FastMath.ceil(maneuver.getDeltaV().getNorm() / maxDV);
                    maneuver.getTrajectory().addManeuver(maneuver.getDate(),
                                                         maneuver.getDeltaV().negate(),
                                                         maneuver.getIsp());

                    // add the various parts of the split maneuver
                    for (int j = 0; j < nbParts; ++j) {
                        final ScheduledManeuver m = new ScheduledManeuver(maneuver.getModel(), maneuver.getDate().shiftedBy(j * orbitsSeparation * period),
                                                                          new Vector3D(1.0 / nbParts, maneuver.getDeltaV()),
                                                                          maneuver.getThrust(),
                                                                          maneuver.getIsp(), maneuver.getTrajectory(),
                                                                          false);
                        m.getTrajectory().addManeuver(m.getDate(), m.getDeltaV(), m.getIsp());
                        modified.add(m);
                    }

                } else {
                    // the maneuver is immune to splitting
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
