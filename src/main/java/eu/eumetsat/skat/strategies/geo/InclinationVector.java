/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.MathUtils;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.AdapterPropagator;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.SecularAndHarmonic;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Station-keeping control for inclination vector.
 * <p>
 * The mean inclination vector moves along a line mainly due to third
 * body attraction with 13 days and 6 months long period variations
 * (half Moon period and half Sun period), and also some short period
 * terms.
 * </p>
 * <p>
 * This control law attempts to control only the secular motion and
 * bring it back to a reference point only when it escapes an allowed
 * circle.
 * </p>
 * <p>
 * The control law simply fits the actual motion with a simplified
 * model with secular and long period terms and when this motion escapes
 * the allowed region, it tries to bring the secular part back to the
 * prescribed point.
 * </p>
 * <p>
 * This control law tunes maneuvers by using an out-of-plane maneuver,
 * which may be split if it exceeds maximal &Delta;V.
 * </p>
 * @author Luc Maisonobe
 */
public class InclinationVector extends AbstractSKControl {

    /** Sun pulsation, 6 months period. */
    private static final double SUN_PULSATION = 4.0 * FastMath.PI / Constants.JULIAN_YEAR;

    /** Moon pulsation (two weeks period, we use synodic period here). */
    private static final double MOON_PULSATION = 4.0 * FastMath.PI / (29.530589 * Constants.JULIAN_DAY);
    
    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Maximum number of maneuvers to set up in one cycle. */
    private final int maxManeuvers;

    /** Time offset of the first maneuver with respect to cycle start. */
    private final double firstOffset;

    /** Minimum time between split parts in number of orbits. */
    private final int orbitsSeparation;

    /** Iteration number. */
    private int iteration;

    /** Abscissa of reference inclination vector. */
    private final double referenceHx;

    /** Ordinate of target inclination vector. */
    private final double referenceHy;

    /** Number of maneuvers to perform. */
    private int nbMan;

    /** Abscissa of start inclination vector before maneuver. */
    private double startHx;

    /** Ordinate of start inclination vector before maneuver. */
    private double startHy;

    /** Abscissa of target inclination vector. */
    private double targetHx;

    /** Ordinate of target inclination vector. */
    private double targetHy;

    /** Radius of the inner circle (internally osculating the limit inclination angle circle). */
    private final double innerRadius;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Reference state at fitting start. */
    private SpacecraftState fitStart;

    /** Model for the x part. */
    private SecularAndHarmonic xModel;

    /** Model for the y part. */
    private SecularAndHarmonic yModel;

    /** Cycle start. */
    private AbsoluteDate cycleStart;

    /** Flag indicating whether maneuvers must be paired (2 half maneuvers separated by 12 hours) or not. */
	private boolean isPairedManeuvers;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model out-of-plane maneuver model
     * @param yawFlipSequence array of pairs containing a day-of-year and the corresponding new status of the yaw flip (0 or 1)
     * @param firstOffset time offset of the first maneuver with respect to cycle start
     * @param maxManeuvers maximum number of maneuvers to set up in one cycle
     * @param orbitsSeparation minimum time between split parts in number of orbits
     * @param referenceHx abscissa of reference inclination vector
     * @param referenceHy ordinate of reference inclination vector
     * @param limitInclination limit inclination angle
     * @param samplingStep step to use for sampling throughout propagation
     * @param horizon time horizon duration
     */
    public InclinationVector(final String name, final String controlledName, final int controlledIndex,
                             final TunableManeuver[] model, int[][] yawFlipSequence, final double firstOffset,
                             final int maxManeuvers, final int orbitsSeparation,
                             final double referenceHx, final double referenceHy,
                             final double limitInclination, final double samplingStep, final double horizon, final boolean isPairedManeuvers) {

        super(name, model, yawFlipSequence, controlledName, controlledIndex, null, -1,
              0.0, innerRadius(referenceHx, referenceHy, limitInclination),
              horizon * Constants.JULIAN_DAY);

        this.stephandler       = new Handler();
        this.firstOffset       = firstOffset;
        this.maxManeuvers      = maxManeuvers;
        this.orbitsSeparation  = orbitsSeparation;
        this.referenceHx       = referenceHx;
        this.referenceHy       = referenceHy;
        this.samplingStep      = samplingStep;
        this.innerRadius       = innerRadius(referenceHx, referenceHy, limitInclination);
        this.isPairedManeuvers = isPairedManeuvers;

        xModel = new SecularAndHarmonic(1, new double[] { SUN_PULSATION, MOON_PULSATION });
        yModel = new SecularAndHarmonic(1, new double[] { SUN_PULSATION, MOON_PULSATION });

        // rough order of magnitudes values for initialization purposes
        xModel.resetFitting(AbsoluteDate.J2000_EPOCH,
                            new double[] {
                                referenceHx, 0.0,     1.0e-4, 1.0e-4, 1.0e-5, 1.0e-5
                            });
        yModel.resetFitting(AbsoluteDate.J2000_EPOCH,
                            new double[] {
                                referenceHy, 1.0e-10, 1.0e-4, 1.0e-4, 1.0e-5, 1.0e-5
                            });

    }
    
    /** Find the inner circle radius.
     *  The inner circle is centered on (referenceHx, referenceHy) and osculating
     *  the circle corresponding to limit inclination angle.
     *  @param refHx abscissa of reference inclination vector
     *  @param refHy ordinate of reference inclination vector
     *  @param limitInclination limit circle radius
     *  @return inner circle radius
     */
    private static double innerRadius(final double refHx, final double refHy, final double limitInclination) {
        final double offCenter = FastMath.hypot(refHx, refHy);
        final double maxRadius = FastMath.tan(limitInclination / 2.);
        return (maxRadius - offCenter);
    }

    /** {@inheritDoc} 
     * @throws OrekitException */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException {

        this.iteration = iteration;
        resetMarginsChecks();

        // select a long maneuver-free interval for fitting
        final AbsoluteDate[] freeInterval = getManeuverFreeInterval(maneuvers, fixedManeuvers, start, end);

        fitStart = propagator.propagate(freeInterval[0]);
        this.cycleStart = start;

        if (iteration == 0) {
            // reconstruct inclination motion model only on first iteration

            // fit secular plus long periods model to inclination
            xModel.resetFitting(freeInterval[0], xModel.getFittedParameters());
            yModel.resetFitting(freeInterval[0], yModel.getFittedParameters());
            for (AbsoluteDate date = freeInterval[0]; date.compareTo(freeInterval[1]) < 0; date = date.shiftedBy(samplingStep)) {
                final EquinoctialOrbit orbit =
                        (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(propagator.propagate(date).getOrbit());
                xModel.addPoint(date, orbit.getHx());
                yModel.addPoint(date, orbit.getHy());
            }

            xModel.fit();
            yModel.fit();

        }

    }

    /** {@inheritDoc} 
     * @throws SkatException */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException, SkatException {

        final ScheduledManeuver[] tuned;
        final AdapterPropagator adapterPropagator = new AdapterPropagator(reference);

        if (iteration == 0) {

            if (getMargins() >= 0) {
                // no constraints violations, we don't perform any maneuvers
                nbMan = 0;
                return tunables;
            }

            // inclination vector at maneuver time
            AbsoluteDate date = cycleStart.shiftedBy(firstOffset);
            startHx = xModel.osculatingValue(date);
            startHy = yModel.osculatingValue(date);

            // inclination vector evolution direction, considering only secular and Sun effects
            // we ignore Moon effects here since they have a too short period
            final double dHXdT = xModel.meanDerivative(date, 1, 1);
            final double dHYdT = yModel.meanDerivative(date, 1, 1);
            final double dHdT  = FastMath.hypot(dHXdT, dHYdT);
            
            // Distance traveled by H vector between now and end-of-life. Increase it a bit to have some margin
            final double margin      = .5;
            final AbsoluteDate simulationEnd = getModel().getEndDate();
            final double deltaHToEnd = dHdT * simulationEnd.durationFrom(fitStart.getDate()) * (1+margin);
            
            // Check for end of life: is there enough time left to travel the entire circle before EoL?
            if (deltaHToEnd >= 2 * innerRadius) {
            	// Normal OOP maneuver: there is time
            	// Target a point on the limit circle
            	// such that trajectory crosses the circle radially 
            	targetHx = referenceHx + dHXdT / dHdT * (-innerRadius);
            	targetHy = referenceHy + dHYdT / dHdT * (-innerRadius);
            } else {
            	// End-of-life case: there is no time
            	// Maybe there is such little time that no maneuver at all is necessary
            	// We are here because the H-vector has been found to exit the circle before
            	// the end of the next control cycle, but not before the end of the simulation...
            	final double endRadius = FastMath.hypot(xModel.osculatingValue(simulationEnd), 
            			                                yModel.osculatingValue(simulationEnd));
            	if(endRadius < innerRadius){
            		nbMan = 0;
            		return tunables;
            	}
            	
                // If a maneuver is necessary, do not go back all the way to the lower edge of
            	// the circle, leave just enough distance to finish life within the inner circle.
                targetHx = referenceHx + dHXdT / dHdT * (innerRadius - deltaHToEnd);
                targetHy = referenceHy + dHYdT / dHdT * (innerRadius - deltaHToEnd);                  
            }

            // Compute the out of plane maneuver required to get this inclination offset
            final double deltaHx     = targetHx - startHx;
            final double deltaHy     = targetHy - startHy;
            final double vs          = fitStart.getPVCoordinates().getVelocity().getNorm();
            final double totalDeltaV = 2 * FastMath.hypot(deltaHx, deltaHy) * vs;
            
            // Yaw-flip thrusters if necessary
            final boolean isYawFlipped     = getYawFlip(fitStart.getDate());
            final TunableManeuver[] models = isYawFlipped ? yawFlipModels(getModels()) : getModels();

            // Find thrusters orientation (North or South)
            int modelIndexNorth = 0;
            int modelIndexSouth = 0;
            if(models.length == 2){
            	if     (thrustSignMomentum(fitStart, models[0]) > 0  &&  thrustSignMomentum(fitStart, models[1]) < 0){
            		modelIndexSouth = 1;
            	}
            	else if(thrustSignMomentum(fitStart, models[0]) < 0  &&  thrustSignMomentum(fitStart, models[1]) > 0){
            		modelIndexNorth = 1;
            	}
            	else{
            		throw new SkatException(SkatMessages.INVALID_OPPOSED_THRUSTERS);
            	}
            }
            final TunableManeuver modelNorth = models[modelIndexNorth];
            final TunableManeuver modelSouth = models[modelIndexSouth];
            
            // compute the local time of the maneuvers
            final EquinoctialOrbit orbit = (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(fitStart.getOrbit());
            final double alphaStart      = orbit.getLM();
            final double alphaMan        = FastMath.atan2(deltaHy, deltaHx);
            final double dAlpha          = MathUtils.normalizeAngle(alphaMan - alphaStart, 2 * FastMath.PI);
            final double n               = fitStart.getKeplerianMeanMotion();
            final double separation      = orbitsSeparation * 2 * FastMath.PI / n;
            AbsoluteDate t0              = fitStart.getDate().shiftedBy(dAlpha / n);
            
            
            if(!isPairedManeuvers){
	            // Single maneuvers case
            	// Compute the number and magnitude of required maneuvers 
	            nbMan  = FastMath.min(maxManeuvers, (int) FastMath.ceil(totalDeltaV / modelNorth.getCurrentDVSup()));
	            final double deltaV = FastMath.min(modelNorth.getCurrentDVSup(), totalDeltaV / nbMan);
	
	            tuned = new ScheduledManeuver[tunables.length + nbMan];
	            System.arraycopy(tunables, 0, tuned, 0, tunables.length);
	            changeTrajectory(tuned, 0, tunables.length, adapterPropagator);	
	            
	            while (t0.durationFrom(cycleStart) < firstOffset) {
	                t0 = t0.shiftedBy(separation);
	            }

	            // add the new maneuvers
	            for (int i = 0; i < nbMan; ++i) {
	                tuned[tunables.length + i] =
	                        new ScheduledManeuver(modelNorth, t0.shiftedBy(i * separation),
	                                              new Vector3D(deltaV, modelNorth.getDirection()),
	                                              modelNorth.getCurrentThrust(), modelNorth.getCurrentISP(),
	                                              adapterPropagator, false);
	            }
            }
            else{
            	// Paired maneuvers case

            	// Take a look at thrusters radial (Z) cross-coupling directions
            	final double radialCouplingNorth = modelNorth.getDirection().getZ();
            	final double radialCouplingSouth = modelSouth.getDirection().getZ();

            	if(radialCouplingSouth*radialCouplingNorth > 0){
            		
            		// Same radial coupling directions, compute ideal DV ratio to balance coupling
            		final double northSouthDvRatio = radialCouplingSouth / radialCouplingNorth;
            		
            		// Limit North and South maximum DVs in function of both the ideal ratio and the thrusters limits
            		double maxDvSouth;
            		double maxDvNorth;
            		
            		if(modelSouth.getCurrentDVSup()*northSouthDvRatio > modelNorth.getCurrentDVSup()){
            			maxDvSouth = modelNorth.getCurrentDVSup() / northSouthDvRatio;
            			maxDvNorth = modelNorth.getCurrentDVSup();
            		}else{
            			maxDvSouth = modelNorth.getCurrentDVSup() / northSouthDvRatio;
            			maxDvNorth = modelNorth.getCurrentDVSup();
            		}
            		
            		// Compute the number of required maneuver pairs 
            		final double deltaVPairMax  = maxDvSouth + maxDvNorth;
            		final int nbPairs           = FastMath.min(maxManeuvers, (int) FastMath.ceil(totalDeltaV / deltaVPairMax));
            		final double lastPairDeltaV = totalDeltaV - (nbPairs-1) * deltaVPairMax ;
            		nbMan                       = 2 * nbPairs;
            		

            		// Create new maneuvers array
            		tuned = new ScheduledManeuver[tunables.length + nbMan];
            		System.arraycopy(tunables, 0, tuned, 0, tunables.length);
            		changeTrajectory(tuned, 0, tunables.length, adapterPropagator);

            		while (t0.durationFrom(cycleStart) < firstOffset) {
            			t0 = t0.shiftedBy(separation);
            		}

            		// Build the maneuver sequence
            		for (int iPair = 0; iPair < nbPairs; ++iPair) {

            			// Normal pairs are set at maximum DV
            			double dvNorth = maxDvNorth;
            			double dvSouth = maxDvSouth;

            			if(iPair == nbPairs-1){
            				// The last pair is smaller, but still balancing radial cross coupling
            				dvNorth = maxDvNorth * lastPairDeltaV / deltaVPairMax;
            				dvSouth = maxDvSouth * lastPairDeltaV / deltaVPairMax;
            			}

            			// Create the two maneuvers of the pair
            			tuned[tunables.length + iPair*2] =
            					new ScheduledManeuver(modelNorth, t0.shiftedBy(iPair * separation),
            							new Vector3D(dvNorth, modelNorth.getDirection()),
            							modelNorth.getCurrentThrust(), modelNorth.getCurrentISP(),
            							adapterPropagator, false);
            			tuned[tunables.length + iPair*2 + 1] =
            					new ScheduledManeuver(modelSouth, t0.shiftedBy(iPair * separation + Constants.JULIAN_DAY / 2),
            							new Vector3D(dvSouth, modelSouth.getDirection()),
            							modelSouth.getCurrentThrust(), modelSouth.getCurrentISP(),
            							adapterPropagator, false);
            		}

            		
            		
            	}else{
            		// If they are opposed or null, it is useless to try to balance the cross coupling.
            		// Just "fill" in the delta-V, pair-by-pair, to make things faster
            		// Different radial coupling direction, use only one thruster, the one will less coupling

            		// Compute the number of required maneuver pairs 
            		final double deltaVPairMax  = modelNorth.getCurrentDVSup() + modelSouth.getCurrentDVSup();
            		final int nbPairs           = FastMath.min(maxManeuvers, (int) FastMath.ceil(totalDeltaV / deltaVPairMax));
            		final double lastPairDeltaV = totalDeltaV - (nbPairs-1) * deltaVPairMax ;
            		if(lastPairDeltaV > modelNorth.getCurrentDVSup()){
            			nbMan = 2 * nbPairs;
            		}else{
            			nbMan = 2 * nbPairs - 1;
            		}

            		// Create new maneuvers array
            		tuned = new ScheduledManeuver[tunables.length + nbMan];
            		System.arraycopy(tunables, 0, tuned, 0, tunables.length);
            		changeTrajectory(tuned, 0, tunables.length, adapterPropagator);

            		while (t0.durationFrom(cycleStart) < firstOffset) {
            			t0 = t0.shiftedBy(separation);
            		}

            		// add the new full paired maneuvers
            		for (int iPair = 0; iPair < nbPairs-1; ++iPair) {
            			tuned[tunables.length + iPair*2] =
            					new ScheduledManeuver(modelNorth, t0.shiftedBy(iPair * separation),
            							new Vector3D(modelNorth.getCurrentDVSup(), modelNorth.getDirection()),
            							modelNorth.getCurrentThrust(), modelNorth.getCurrentISP(),
            							adapterPropagator, false);
            			tuned[tunables.length + iPair*2 + 1] =
            					new ScheduledManeuver(modelSouth, t0.shiftedBy(iPair * separation + Constants.JULIAN_DAY / 2),
            							new Vector3D(modelSouth.getCurrentDVSup(), modelSouth.getDirection()),
            							modelSouth.getCurrentThrust(), modelSouth.getCurrentISP(),
            							adapterPropagator, false);
            		}
            		// add the last pair (1 or 2 maneuvers)
            		final double deltaVPos = FastMath.min(modelNorth.getCurrentDVSup(), lastPairDeltaV);
            		final int iPair = nbPairs-1;
            		tuned[tunables.length + iPair*2] =
            				new ScheduledManeuver(modelNorth, t0.shiftedBy(iPair * separation),
            						new Vector3D(deltaVPos, modelNorth.getDirection()),
            						modelNorth.getCurrentThrust(), modelNorth.getCurrentISP(),
            						adapterPropagator, false);
            		if(lastPairDeltaV > modelNorth.getCurrentDVSup()){
            			final double deltaVNeg = FastMath.min(modelSouth.getCurrentDVSup(), lastPairDeltaV - modelNorth.getCurrentDVSup());
            			tuned[tunables.length + iPair*2 + 1] =
            					new ScheduledManeuver(modelSouth, t0.shiftedBy(iPair * separation + Constants.JULIAN_DAY / 2),
            							new Vector3D(deltaVNeg, modelSouth.getDirection()),
            							modelSouth.getCurrentThrust(), modelSouth.getCurrentISP(),
            							adapterPropagator, false);
            		}
            	}
            }
        } else {

            if (nbMan == 0) {
                // no maneuvers are needed
                return tunables;
            }

            // adjust the existing maneuvers
        	final TunableManeuver[] models = getModels();
        	
            // find the date of the last adjusted maneuver
        	ScheduledManeuver last = null;
        	for (final ScheduledManeuver maneuver : tunables) {
        		for (final TunableManeuver model : models){
        			if (maneuver.getName().equals(model.getName())) {
        				if (last == null || maneuver.getDate().compareTo(last.getDate()) > 0) {
        					last = maneuver;
        				}
        			}
        		}
        	}

            // achieved inclination after the last maneuver
            final EquinoctialOrbit orbit =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(last.getStateAfter().getOrbit());
            final double achievedHx = orbit.getHx();
            final double achievedHy = orbit.getHy();

            // velocity increment correction needed to reach target
            final double ratio  = FastMath.hypot(targetHx   - startHx, targetHy   - startHy) /
                                  FastMath.hypot(achievedHx - startHx, achievedHy - startHy);
            final double deltaV = nbMan * last.getSignedDeltaV() * (ratio - 1.0);

            // time offset needed to reach target
            final double deltaAlpha =
                    MathUtils.normalizeAngle(FastMath.atan2(targetHy   - startHy, targetHx   - startHx) -
                                             FastMath.atan2(achievedHy - startHy, achievedHx - startHx),
                                             0.0);
            final double deltaT = deltaAlpha / orbit.getKeplerianMeanMotion();

            // distribute the change over all maneuvers
            tuned = tunables.clone();
            changeTrajectory(tuned, 0, tuned.length, adapterPropagator);
            distributeDV(deltaV, deltaT, tuned, adapterPropagator);

        }

        // finalize propagator
        for (final ScheduledManeuver maneuver : tuned) {
            adapterPropagator.addEffect(new SmallManeuverAnalyticalModel(maneuver.getStateBefore(),
                                                                         maneuver.getDeltaV(),
                                                                         maneuver.getIsp()));
        }

        return tuned;

    }


    private TunableManeuver[] yawFlipModels(final TunableManeuver[] models){
    	TunableManeuver[] flippedModels = new TunableManeuver[models.length];
    	for(int i=0; i<models.length; i++){
    		TunableManeuver model     = models[i];
    		Vector3D direction        = model.getDirection();
    		Vector3D flippedDirection = new Vector3D(-direction.getX(), -direction.getY(), direction.getZ());
    		flippedModels[i]          = new TunableManeuver(model, flippedDirection);
    	}
    	return flippedModels;
    }
    
    /** {@inheritDoc} */
    public EventDetector getEventDetector() {
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return stephandler;
    }

    /** Inner class for step handling. */
    private class Handler implements OrekitStepHandler {

        /** Serializable UID. */
        private static final long serialVersionUID = 8803174499877772678L;

        /** {@inheritDoc} */
        public void init(final SpacecraftState s0, final AbsoluteDate t) {
        }

        /** {@inheritDoc} */
        public void handleStep(OrekitStepInterpolator interpolator, boolean isLast)
            throws PropagationException {

            try {

                // find step boundaries
                final AbsoluteDate minDate =
                        interpolator.isForward() ? interpolator.getPreviousDate() : interpolator.getCurrentDate();
                final AbsoluteDate maxDate =
                        interpolator.isForward() ? interpolator.getCurrentDate() : interpolator.getPreviousDate();

                // loop throughout step
                for (AbsoluteDate date = minDate; date.compareTo(maxDate) < 0; date = date.shiftedBy(samplingStep)) {

                    interpolator.setInterpolatedDate(date);
                    final SpacecraftState state = interpolator.getInterpolatedState();

                    // check limit circle violations
                    final double radius = FastMath.hypot(state.getHx() - referenceHx, state.getHy() - referenceHy);
                    checkMargins(date, radius);

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
