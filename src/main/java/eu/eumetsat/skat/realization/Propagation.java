/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.List;

import org.apache.commons.math.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.ConstantThrustManeuver;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.OrbitType;
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

    /** Orbit propagators. */
    private final Propagator[] propagators;

    /** Indicator for compensating long burns inefficiency. */
    private boolean compensateLongBurn;

    /** Cycle end date. */
    private AbsoluteDate cycleEnd;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param propagators propagators to use for each spacecraft
     * @param compensateLongBurn if true, long burn inefficiency should be compensated
     */
    public Propagation(final int[] spacecraftIndices, final Propagator[] propagators,
                       final boolean compensateLongBurn) {
        this.spacecraftIndices  = spacecraftIndices.clone();
        this.propagators        = propagators.clone();
        this.compensateLongBurn = compensateLongBurn;
    }

    /** {@inheritDoc} */
    public void setCycleEnd(final AbsoluteDate cycleEnd) {
        this.cycleEnd = cycleEnd;
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException, SkatException {

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
            propagators[i].clearEventsDetectors();
            for (final ScheduledManeuver maneuver : performed) {
                final Propagator p = maneuver.getTrajectory();
                final double nominalDuration = maneuver.getDuration(p.propagate(maneuver.getDate()).getMass());
                final double inefficiency;
                if (compensateLongBurn && (!maneuver.isInPlane())) {
                    // this is a long out of plane maneuver, we adapt Isp to reflect
                    // the fact more mass will be consumed to achieve the same velocity increment

                    final SpacecraftState startState = p.propagate(maneuver.getDate().shiftedBy(-0.5 * nominalDuration));
                    final CircularOrbit startOrbit   = (CircularOrbit) (OrbitType.CIRCULAR.convertType(startState.getOrbit()));
                    final double alphaS              = startOrbit.getAlphaV();

                    final SpacecraftState endState   = p.propagate(maneuver.getDate().shiftedBy(0.5 * nominalDuration));
                    final CircularOrbit endOrbit     = (CircularOrbit) (OrbitType.CIRCULAR.convertType(endState.getOrbit()));
                    final double alphaE              = endOrbit.getAlphaV();

                    inefficiency = (FastMath.sin(alphaE) - FastMath.sin(alphaS)) / (alphaE - alphaS);

                } else {
                    inefficiency = 1.0;
                }
                if (propagators[i] instanceof NumericalPropagator) {
                    ((NumericalPropagator) propagators[i]).addForceModel(new ConstantThrustManeuver(maneuver.getDate().shiftedBy(-0.5 * nominalDuration),
                                                                                                    nominalDuration,
                                                                                                    maneuver.getThrust(),
                                                                                                    maneuver.getIsp(),
                                                                                                    maneuver.getDeltaV().normalize()));
                } else {
                    propagators[i].addEventDetector(new ImpulseManeuver(new DateDetector(maneuver.getDate()),
                                                                        maneuver.getDeltaV(),
                                                                        inefficiency * maneuver.getIsp()));
                }
            }
            propagators[i].setEphemerisMode();

            // perform propagation
            propagators[i].resetInitialState(originals[index].getRealStartState());
            updated[index] = originals[index].updateRealEndState(propagators[i].propagate(cycleEnd));

            // retrieve continuous data
            updated[index] = updated[index].updatePerformedEphemeris(propagators[i].getGeneratedEphemeris());

        }

        return updated;

    }

}
