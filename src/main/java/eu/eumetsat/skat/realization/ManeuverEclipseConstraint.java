/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.AdapterPropagator;
import org.orekit.propagation.events.EclipseDetector;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
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
    private final double[][] entryDelay;

    /** Time margin before eclipse exit. */
    private final double[][] exitDelay;

    /** Number of orbits between parts of a split maneuver. */
    private final int nbOrbits;

    /** Minimum duration ratio w.r.t. eclipse duration (taking margins into account). */
    private final double minEclipseRatio;

    /** Indicator for compensating inefficiency due to (1) out-of-plane maneuver asymmetry w.r.t. ascending or descending node location
     *  and extended maneuver. */
    private final boolean compensateLongBurnAndNodeAsymmetry;

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
     * @param minEclipseRatio minimum ratio of maneuver duration wrt eclipse duration
     * @param compensateNodeAsymmetry indicator for compensating inefficiency due to out-of-plane maneuver asymmetry w.r.t. ascending or descending node location.
     * @param sun Sun model
     * @param earth Earth model
     * @throws SkatException 
     */
    public ManeuverEclipseConstraint(final int[] spacecraftIndices, final String name,
                                     final double[][] entryDelay, final double[][] exitDelay,
                                     final int nbOrbits, final double minEclipseRatio,
                                     final boolean compensateLongBurnAndNodeAsymmetry, 
                                     final CelestialBody sun,
                                     final OneAxisEllipsoid earth)
        throws IllegalArgumentException, SkatException {
        this.spacecraftIndices        = spacecraftIndices.clone();
        this.name                     = name;
        this.nbOrbits      		      = nbOrbits;
        this.minEclipseRatio		  = minEclipseRatio;
        this.compensateLongBurnAndNodeAsymmetry  = compensateLongBurnAndNodeAsymmetry;
        this.sun             		  = sun;
        this.earth           		  = earth;
        
        this.entryDelay = entryDelay.clone();
        for (int i = 1; i < entryDelay.length; ++i) {
            if (entryDelay[i - 1][1] > entryDelay[i][1]) {
                throw new SkatException(SkatMessages.NON_INCREASING_DATES_IN_ENTRY_DELAY_CURVE,
                		entryDelay[i - 1][1], entryDelay[i][1]);
            }
        }
        this.exitDelay = exitDelay.clone();
        for (int i = 1; i < exitDelay.length; ++i) {
            if (exitDelay[i - 1][1] > exitDelay[i][1]) {
                throw new SkatException(SkatMessages.NON_INCREASING_DATES_IN_EXIT_DELAY_CURVE,
                		exitDelay[i - 1][1], exitDelay[i][1]);
            }
        }
        
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
                    final AbsoluteDate manoCentralDate = maneuver.getDate();
                    final SpacecraftState state        = maneuver.getStateBefore();
                    double manoDuration                = maneuver.getDuration(state.getMass());
                    final AbsoluteDate manoStart       = manoCentralDate.shiftedBy(-0.5 * manoDuration);
                    final AbsoluteDate manoEnd         = manoCentralDate.shiftedBy( 0.5 * manoDuration);

                    // find the closest eclipse
                    final double period            = state.getKeplerianPeriod();
                    final EclipseSelector selector = new EclipseSelector(state.getFrame(), manoCentralDate);
                    final Propagator tmpPropagator = new AdapterPropagator(maneuver.getTrajectory());
                    tmpPropagator.addEventDetector(selector);
                    tmpPropagator.propagate(manoStart.shiftedBy(-1.5 * period), manoEnd.shiftedBy(1.5 * period));
                    if ((selector.getEntry() == null) || (selector.getExit() == null)) {
                        throw new SkatException(SkatMessages.NO_ECLIPSE_AROUND_DATE, manoCentralDate);
                    }

                    // Compute earliest and latest maneuver allowed times due to eclipse (including margins) 
                    final double consumedMass          = originals[index].getBOLMass() - state.getMass();
                    final AbsoluteDate earliestAllowed = selector.getEntry().shiftedBy(getEntryDelay(consumedMass));
                    final AbsoluteDate latestAllowed   = selector.getExit().shiftedBy(-getExitDelay(consumedMass));
                    // Compute maximum single maneuver duration due to eclipse
                    final double maxSingleBurnDuration = latestAllowed.durationFrom(earliestAllowed);
                    // Compute central date of eclipse
                    final AbsoluteDate centralEclipseDate = earliestAllowed.shiftedBy(0.5*maxSingleBurnDuration);

                    // if compensation due to asymmetry w.r.t. node needs to be applied
                    if (compensateLongBurnAndNodeAsymmetry) {

                    	// get new duration assuming maneuver duration equal to effective eclipse duration
                    	manoDuration = compensateDurationLongBurnAndNodeAssymetry(manoDuration, tmpPropagator, earliestAllowed, latestAllowed);
                    	
                    }

                    // compute number of full maneuvers (assuming maximum duration)
                    double nbParts = (int) FastMath.ceil(manoDuration / maxSingleBurnDuration);
                    
                    // compute eclipse ratio if all maneuvers are equal
                    double eclipseRatio = (manoDuration/nbParts)/maxSingleBurnDuration;
                	double lostEclipseRatio = 0;

                    
                    // if this eclipse ratio is smaller than minimum eclipse ratio	
                    if (eclipseRatio<minEclipseRatio) {
                    	
                    	// reduce number of maneuvers by one
                    	nbParts = nbParts - 1;
                    	
                    	// eclipse ratio is equal to one
                        eclipseRatio = 1.0;
                    	
                    	// lost eclipse ratio
                    	lostEclipseRatio = manoDuration / maxSingleBurnDuration - nbParts;

                    }
                                       	
                    // remove the original maneuver from the trajectory
                    maneuver.getTrajectory().addEffect(new SmallManeuverAnalyticalModel(maneuver.getStateBefore(),
                                                                                        maneuver.getDeltaV().negate(),
                                                                                        -maneuver.getIsp()));
                    // add all maneuvers
                    for (int j = 0; j < nbParts; ++j) {
                    	
                        ScheduledManeuver m = new ScheduledManeuver(maneuver.getModel(),
							      										  centralEclipseDate.shiftedBy(j * nbOrbits * period),
                                                                          new Vector3D(maneuver.getSignedDeltaV() / nbParts, maneuver.getModel().getDirection()),
                                                                          maneuver.getThrust(), maneuver.getIsp(),
                                                                          maneuver.getTrajectory(), false);
                        // if long burn and node asymmetry compensation
                        if (compensateLongBurnAndNodeAsymmetry) {
                        	m = longBurnAndNodeAssymetryCompensation(m);
                        }
                        m.getTrajectory().addEffect(new SmallManeuverAnalyticalModel(m.getStateBefore(),
                                                    m.getDeltaV(),
                                                    m.getIsp()));
                        
                        // set eclipse ratio
                        m.updateEclipseRatio(eclipseRatio);
                        
                        // set maneuver loss due to eclipse constraint
                    	m.updateLostEclipseRatio(lostEclipseRatio);

                        // add maneuver
                        modified.add(m);

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

    /** Increase maneuver duration due to inefficiency of long burns and node assymetry.
     *  <p>
     *  Maneuver duration is recomputed to be consider during the segmenting approach
     *  </p>
     *  @param duration initial duration
     *  @param propag propagator
     *  @param start start absolute date
     *  @param end end absolute date
     *  @return compensated maneuver
     * @throws OrekitException 
     */
    private double compensateDurationLongBurnAndNodeAssymetry(final double duration, final Propagator propag, final AbsoluteDate start, final AbsoluteDate end) throws OrekitException {

    	// get PSO (Position Sur l'Orbite) at start
    	PVCoordinates pvCoord = propag.getPVCoordinates(start, FramesFactory.getMOD(false));
    	CartesianOrbit carOrbit = new CartesianOrbit(pvCoord,FramesFactory.getMOD(false),start, Constants.WGS84_EARTH_MU);
        KeplerianOrbit kepOrbit = (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(carOrbit);
        double psoStart = kepOrbit.getMeanAnomaly() + kepOrbit.getPerigeeArgument();
        
    	// get PSO (Position Sur l'Orbite) at end
    	pvCoord = propag.getPVCoordinates(end, FramesFactory.getMOD(false));
    	carOrbit = new CartesianOrbit(pvCoord,FramesFactory.getMOD(false),end, Constants.WGS84_EARTH_MU);
        kepOrbit = (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(carOrbit);
        double psoEnd = kepOrbit.getMeanAnomaly() + kepOrbit.getPerigeeArgument();

        // get delta PSO
        double alphaDelta                = psoEnd - psoStart;
        while( alphaDelta < -FastMath.PI) alphaDelta+=2*FastMath.PI;
        
        // get increase factor
        final double increaseFactor     = (FastMath.sin(psoEnd) - FastMath.sin(psoStart)) / alphaDelta;
        
        // return new duration
        return duration/increaseFactor;

    }
    
    
    public double getEntryDelay(final double consumedMass) {
    	return(interpolateDelay(consumedMass, entryDelay));
    }
 
    public double getExitDelay(final double consumedMass) {
    	return(interpolateDelay(consumedMass, exitDelay));
    }
 
    private double interpolateDelay(final double consumedMass, final double[][] delayArray) {

    	double delay = delayArray[0][1];
        if(consumedMass >= delayArray[delayArray.length-1][1]) {
            // mass is greater than last curve point,
            // we are in a regulated phase, delay is constant
            delay = delayArray[delayArray.length-1][0];
        }
        else{
	        for (int i = delayArray.length-1; i > 0 ; --i) {
	            if (consumedMass >= delayArray[i-1][1]) {
	                // we are in an interval between two curve points
	                // we are in blow-down mode, delay evolves linearly
	                final double delay0 = delayArray[i - 1][0];
	                final double mass0  = delayArray[i - 1][1];
	                final double delay1 = delayArray[i][0];
	                final double mass1  = delayArray[i][1];
	                delay = (delay0 * (consumedMass - mass1) + delay1 * (mass0 - consumedMass)) /
	                                (mass0 - mass1);
	                break;
	            }
	        }
        }
        
        return(delay);
    }
    
    
    
    /** Compensate inefficiency of long burns and node assymetry.
     *  <p>
     *  For a long out-of-plane maneuver, Isp has to be adapted to reflect the
     *  fact more mass will be consumed to achieve the same velocity increment.
     *  </p>
     *  @param maneuver maneuver to compensate
     *  @return compensated maneuver (Isp reduced to get same dV with more consumed mass)
     *  @throws PropagationException if state cannot be propagated around maneuvera
     */
    private ScheduledManeuver longBurnAndNodeAssymetryCompensation(final ScheduledManeuver maneuver) throws PropagationException {
        // this is a long out of plane maneuver, we adapt Isp to reflect
        // the fact more mass will be consumed to achieve the same velocity increment
        final double nominalDuration     = maneuver.getDuration(maneuver.getStateBefore().getMass());

        final SpacecraftState startState = maneuver.getState(-0.5 * nominalDuration);
        final CircularOrbit startOrbit   = (CircularOrbit) (OrbitType.CIRCULAR.convertType(startState.getOrbit()));
        final double alphaS              = startOrbit.getAlphaV();

        final SpacecraftState endState   = maneuver.getState(+0.5 * nominalDuration);
        final CircularOrbit endOrbit     = (CircularOrbit) (OrbitType.CIRCULAR.convertType(endState.getOrbit()));
        final double alphaE              = endOrbit.getAlphaV();

        double alphaDelta                = alphaE - alphaS;
        while( alphaDelta < -FastMath.PI) alphaDelta+=2*FastMath.PI;
        final double reductionFactor     = (FastMath.sin(alphaE) - FastMath.sin(alphaS)) / alphaDelta;

        return new ScheduledManeuver(maneuver.getModel(),
                                     maneuver.getDate(),
                                     maneuver.getDeltaV(),
                                     maneuver.getThrust(),
                                     reductionFactor * maneuver.getModel().getCurrentISP(),
                                     maneuver.getTrajectory(),
                                     maneuver.isReplanned());
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
