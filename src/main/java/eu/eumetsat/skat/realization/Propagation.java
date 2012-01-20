/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.List;

import org.orekit.errors.OrekitException;
import org.orekit.forces.ForceModel;
import org.orekit.forces.maneuvers.ConstantThrustManeuver;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.DateDetector;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;
import eu.eumetsat.skat.utils.SupportedPropagator.PropagatorRandomizer;

/**
 * Class for simple propagation of a station-keeping cycle.
 * <p>
 * This class performs propagation of real spacecraft state, using
 * the maneuver that have been determined in the control part of the
 * simulation.
 * </p>
 * @author Luc Maisonobe
 */
public class Propagation implements ScenarioComponent {

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Orbit propagator randomizers. */
    private final PropagatorRandomizer[] randomizers;

    /** Name of the maneuver triggering cycle truncation (may be null). */
    private final String truncationManeuverName;

    /**  Truncation delay after maneuver. */
    private final double truncationManeuverDelay;

    /** Cycle end date. */
    private AbsoluteDate cycleEnd;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param randomizers orbit propagator randomizers to use for each spacecraft
     * @param truncationManeuverName name of the maneuver triggering cycle truncation (may be null)
     * @param truncationManeuverDelay truncation delay after maneuver
     */
    public Propagation(final int[] spacecraftIndices, final PropagatorRandomizer[] randomizers,
                       final String truncationManeuverName, final double truncationManeuverDelay) {
        this.spacecraftIndices       = spacecraftIndices.clone();
        this.randomizers             = randomizers.clone();
        this.truncationManeuverName  = truncationManeuverName;
        this.truncationManeuverDelay = truncationManeuverDelay;
    }

    /** {@inheritDoc} */
    public void setCycleEnd(final AbsoluteDate cycleEnd) {
        this.cycleEnd = cycleEnd;
    }

    public Propagator getPropagator(int i, SpacecraftState s) throws OrekitException {
        return randomizers[i].getPropagator(s);
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException, SkatException {

        // truncate cycle if needed after last maneuver of some type (as they may appear in groups)
        ScheduledManeuver latestManeuver = null;
        if (truncationManeuverName != null) {
            for (int i = 0; i < spacecraftIndices.length; ++i) {
                final int index = spacecraftIndices[i];
                for (final ScheduledManeuver maneuver : originals[index].getManeuvers()) {
                    if (maneuver.getName().equals(truncationManeuverName) &&
                        (latestManeuver == null || maneuver.getDate().compareTo(latestManeuver.getDate()) > 0)) {
                        latestManeuver = maneuver;
                    }
                }
            }
        }
        if (latestManeuver != null) {
            final AbsoluteDate truncationDate =
                    latestManeuver.getDate().shiftedBy(truncationManeuverDelay);
            if (cycleEnd.compareTo(truncationDate) > 0) {
                cycleEnd = truncationDate;
            }
        }

        final ScenarioState[] updated = new ScenarioState[originals.length];

        // separately propagate each spacecraft
        for (int i = 0; i < spacecraftIndices.length; ++i) {

            // select the current spacecraft affected by this component
            final int index = spacecraftIndices[i];
            final List<ScheduledManeuver> performed = originals[index].getManeuvers();
            if (performed == null) {
                throw new SkatException(SkatMessages.NO_MANEUVERS_STATE,
                                        originals[index].getName(), originals[index].getCyclesNumber());
            }

            // set up the propagator with the maneuvers to perform
            final Propagator propagator =
                    randomizers[i].getPropagator(originals[index].getRealState());

            for (final ScheduledManeuver maneuver : performed) {
                if (maneuver.getDeltaV().getNorm() > 1.0e-10) {
                    final double nominalDuration = maneuver.getDuration(maneuver.getStateBefore().getMass());
                    if (propagator instanceof NumericalPropagator) {
                        final ForceModel ctm = new ConstantThrustManeuver(maneuver.getDate().shiftedBy(-0.5 * nominalDuration),
                                                                          nominalDuration,
                                                                          maneuver.getThrust(),
                                                                          maneuver.getIsp(),
                                                                          maneuver.getDeltaV().normalize());
                        ((NumericalPropagator) propagator).addForceModel(ctm);
                    } else {
                        propagator.addEventDetector(new ImpulseManeuver(new DateDetector(maneuver.getDate()),
                                                                        maneuver.getDeltaV(),
                                                                        maneuver.getIsp()));
                    }
                }
            }
            propagator.setEphemerisMode();

            // perform propagation
            updated[index] = originals[index].updateRealState(propagator.propagate(cycleEnd));

            // retrieve continuous data
            updated[index] = updated[index].updatePerformedEphemeris(propagator.getGeneratedEphemeris());

        }

        return updated;

    }

}
