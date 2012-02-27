/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;
import org.orekit.frames.Frame;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.AdapterPropagator;
import org.orekit.propagation.events.EclipseDetector;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.PVCoordinatesProvider;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Class for splitting maneuvers in case of eclipse constraints.
 * @author Luc Maisonobe
 */
public class ManeuverEclipseConstraint implements ScenarioComponent {

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Name of maneuvers to which this component applies. */
    private final String name;

    /** Time margin after eclipse entry. */
    private final double entryDelay;

    /** Time margin before eclipse exit. */
    private final double exitDelay;

    /** Number of orbits between parts of a split maneuver. */
    private final int nbOrbits;

    /** Minimal duration ratio. */
    private final double durationMaxRatio;

    /** Sun model. */
    private final CelestialBody sun;

    /** Earth model. */
    private final OneAxisEllipsoid earth;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param name name of maneuvers to which this component applies
     * @param entryDelay time margin after eclipse entry
     * @param exitDelay time margin before eclipse exit
     * @param nbOrbits number of orbits between parts of a split maneuver
     * @param durationMaxRatio maximal ratio of maneuver duration wrt eclipse duration
     * @param sun Sun model
     * @param earth Earth model
     */
    public ManeuverEclipseConstraint(final int[] spacecraftIndices, final String name,
                                     final double entryDelay, final double exitDelay,
                                     final int nbOrbits, final double durationMaxRatio,
                                     final CelestialBody sun,
                                     final OneAxisEllipsoid earth)
        throws IllegalArgumentException {
        this.spacecraftIndices = spacecraftIndices.clone();
        this.name              = name;
        this.entryDelay        = entryDelay;
        this.exitDelay         = exitDelay;
        this.nbOrbits          = nbOrbits;
        this.durationMaxRatio  = durationMaxRatio;
        this.sun               = sun;
        this.earth             = earth;
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

                    // maneuvers limits
                    final AbsoluteDate centralDate = maneuver.getDate();
                    final SpacecraftState state    = maneuver.getStateBefore();
                    final double burnDuration      = maneuver.getDuration(state.getMass());
                    final AbsoluteDate burnStart   = centralDate.shiftedBy(-0.5 * burnDuration);
                    final AbsoluteDate burnEnd     = centralDate.shiftedBy( 0.5 * burnDuration);

                    // find the closest eclipse
                    final double period            = state.getKeplerianPeriod();
                    final EclipseSelector selector = new EclipseSelector(state.getFrame(), centralDate);
                    final Propagator tmpPropagator = new AdapterPropagator(maneuver.getTrajectory());
                    tmpPropagator.addEventDetector(selector);
                    tmpPropagator.propagate(burnStart.shiftedBy(-1.5 * period), burnEnd.shiftedBy(1.5 * period));
                    if ((selector.getEntry() == null) || (selector.getExit() == null)) {
                        throw new SkatException(SkatMessages.NO_ECLIPSE_AROUND_DATE, centralDate);
                    }

                    // find the number of parts into which the maneuver will be split
                    int nbParts = 1;

                    // Check maneuver start
                    final AbsoluteDate earliestAllowed = selector.getEntry().shiftedBy(entryDelay);
                    if (centralDate.compareTo(earliestAllowed) < 0) {
                        throw new SkatException(SkatMessages.NO_ECLIPSE_AROUND_DATE, centralDate);
                    } else if (burnStart.compareTo(earliestAllowed) < 0) {
                        final double maxDuration = 2.0 * centralDate.durationFrom(earliestAllowed);
                        nbParts = FastMath.max(nbParts, (int) FastMath.ceil(burnDuration / maxDuration));
                    }

                    // Check maneuver end
                    final AbsoluteDate latestAllowed   = selector.getExit().shiftedBy(-exitDelay);
                    if (centralDate.compareTo(latestAllowed) > 0) {
                        throw new SkatException(SkatMessages.NO_ECLIPSE_AROUND_DATE, centralDate);
                    } else if (burnEnd.compareTo(latestAllowed) > 0) {
                        final double maxDuration = 2.0 * latestAllowed.durationFrom(centralDate);
                        nbParts = FastMath.max(nbParts, (int) FastMath.ceil(burnDuration / maxDuration));
                    }

                    // Check maneuver duration
                    final double maxDuration = durationMaxRatio * latestAllowed.durationFrom(earliestAllowed);
                    if (maxDuration < burnDuration) {
                        nbParts = FastMath.max(nbParts, (int) FastMath.ceil(burnDuration / maxDuration));
                    }

                    if (nbParts == 1) {
                        // the maneuver is unchanged
                        modified.add(maneuver);
                    } else {
                        // remove the original maneuver from the trajectory
                        maneuver.getTrajectory().addEffect(new SmallManeuverAnalyticalModel(maneuver.getStateBefore(),
                                                                                            maneuver.getDeltaV().negate(),
                                                                                            maneuver.getIsp()));

                        // add the various parts of the split maneuver
                        final double dV = maneuver.getSignedDeltaV() / nbParts;
                        for (int j = 0; j < nbParts; ++j) {
                            final ScheduledManeuver m = new ScheduledManeuver(maneuver.getModel(),
                                                                              centralDate.shiftedBy(j * nbOrbits * period),
                                                                              new Vector3D(dV, maneuver.getModel().getDirection()),
                                                                              maneuver.getThrust(), maneuver.getIsp(),
                                                                              maneuver.getTrajectory(), false);
                            m.getTrajectory().addEffect(new SmallManeuverAnalyticalModel(m.getStateBefore(),
                                                                                         m.getDeltaV(),
                                                                                         m.getIsp()));
                            modified.add(m);
                        }
                    }

                } else {
                    // the maneuver is immune to eclipse constraint
                    modified.add(maneuver);
                }
            }

            // update the state
            updated[index] = originals[index].updateManeuvers(modified);

        }

        // return an updated states
        return updated;

    }

    /** Selector for eclipse close to a specified date. */
    private class EclipseSelector extends EclipseDetector {

        /** Serializble UID. */
        private static final long serialVersionUID = 666564044264536447L;

        /** Central date expected to be within eclipse. */
        private final AbsoluteDate central;

        /** Entry of the eclipse closest to central date. */
        private AbsoluteDate entry;

        /** Exit of the eclipse closest to central date. */
        private AbsoluteDate exit;

        /** Simple constructor.
         * @param earthCenteredFrame Earth centered inertial frame
         * @param central central date expected to be close to eclipse
         */
        private EclipseSelector(final Frame earthCenteredFrame, final AbsoluteDate central) {
            super(sun, Constants.SUN_RADIUS, new PVCoordinatesProvider() {
                
                /** {@inheritDoc} */
                public PVCoordinates getPVCoordinates(AbsoluteDate date, Frame frame)
                    throws OrekitException {
                    return earthCenteredFrame.getTransformTo(frame, date).transformPVCoordinates(PVCoordinates.ZERO);
                }

            }, earth.getEquatorialRadius(), true);
            this.central = central;
            this.entry   = null;
            this.exit    = null;
        }

        /** {@inheritDoc} */
        @Override
        public Action eventOccurred(final SpacecraftState s, final boolean increasing) {

            if (increasing) {
                // this is an eclipse exit
                if ((entry != null) && (exit == null)) {
                    // store the exit associated with current selected entry
                    exit = s.getDate();
                }
            } else {
                // this is an eclipse entry
                if ((exit == null) ||
                    (FastMath.abs(s.getDate().durationFrom(central)) <= FastMath.abs(exit.durationFrom(central)))) {
                    // this is the start of the closest eclipse found until now
                    entry = s.getDate();
                    exit  = null;
                }
            }

            // continue propagation
            return Action.CONTINUE;

        }

        /** Get the entry of the eclipse closest to central date.
         * @return eclipse entry (null if none found)
         */
        public AbsoluteDate getEntry() {
            return entry;
        }

        /** Get exit of the eclipse closest to central date.
         * @return eclipse exit (null if none found)
         */
        public AbsoluteDate getExit() {
            return exit;
        }

    }

}
