/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import java.util.ArrayList;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.MathUtils;
import org.orekit.bodies.CelestialBody;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;
import org.orekit.frames.FramesFactory;
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
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.SecularAndHarmonic;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;

/**
 * Station-keeping control attempting to follow perigee solar pointing
 * with respect to a specified eccentricity circle.
 * <p>
 * The mean eccentricity vector moves along a circle mainly due to radiation
 * pressure. In addition to this mean motion, third body attraction adds
 * a lot of short period variations.
 * </p>
 * <p>
 * This control law attempts to control only the mean motion along a
 * user prescribed circle with center (c<sub>x</sub>, c<sub>y</sub>)
 * and radius r. If the radius is different from the natural radius
 * induces by radiation pressure, the control should split the natural
 * motion in different pieces and spread these pieces on the prescribed
 * circle. However, due to short periods, this piecewise reconstruction
 * is not very clear and appears as a circular points clouds with many
 * overlapping parts. This is normal behavior.
 * </p>
 * <p>
 * The control law simply computes for each observed point the theoretical
 * circle center by subtracting the natural offset (r cos &alpha;<sub>s</sub>,
 * r sin &alpha;<sub>s</sub>) where &alpha;<sub>s</sub> is the Sun right
 * ascension and r is the prescribed radius. This has the effect of computing
 * a reference point which is independent of the date. Maneuvers are then
 * computed so this reconstructed center is put at the user specified location.
 * </p>
 * <p>
 * This control law tunes maneuvers by selecting the last pair of in-plane
 * maneuvers from the current maneuver set, and changing their relative
 * size and date to change eccentricity, but <em>without</em> changing the
 * overall &Delta;V (i.e. the algebraic sum of the existing maneuvers. The
 * rationale is the the overall &Delta;V should be tuned by the longitude
 * control law (see for example {@link ParabolicLongitude}), and only changes
 * that do not change drastically the longitude are allowed. If there is only
 * one maneuver available (which is the case for example on the first iteration,
 * since longitude control often generates only one maneuver), then the maneuver
 * is split in two. If there are no maneuvers at all, a pair of maneuvers with
 * a zero algebraic sum is created.
 * </p>
 * @author Luc Maisonobe
 */
public class EccentricityCircle extends AbstractSKControl {

    /** Earth pulsation, one day period. */
    private static final double EARTH_PULSATION = 2.0 * FastMath.PI / Constants.JULIAN_DAY;

    /** Sun pulsation, one year period. */
    private static final double SUN_PULSATION = 2.0 * FastMath.PI / Constants.JULIAN_YEAR;

    /** Moon pulsation (one Moon synodic period). */
    private static final double MOON_PULSATION = 2.0 * FastMath.PI / (29.530589 * Constants.JULIAN_DAY);

    /** Average delta-V spent to control inclination [m/s/year] (Soop table 3 lowest value)*/
    private static final double MEAN_OOP_DV_PER_YEAR = 40.6;
	
    /** Associated step handler. */
    private final OrekitStepHandler stephandler;

    /** Abscissa of the circle center. */
    private final double centerX;

    /** Ordinate of the circle center. */
    private final double centerY;

    /** Target circle radius. */
    private final double meanRadius;

    /** Flag for single burn. */
    private final boolean singleBurn;

    /** Sun model. */
    private CelestialBody sun;

    /** Step to use for sampling throughout propagation. */
    private final double samplingStep;

    /** Start of sampling. */
    private SpacecraftState fitStart;

    /** Fitter for x part of eccentricity. */
    private SecularAndHarmonic xModel;

    /** Fitter for x part of eccentricity. */
    private SecularAndHarmonic yModel;

    /** Cycle start. */
    private AbsoluteDate cycleStart;

    /** Time horizon end. */
    private AbsoluteDate timeHorizonEnd;

    /** Inclination control maneuver which X-coupling should be anticipated for. */
	private TunableManeuver oopManeuver;

    /** Date of the last OOP maneuver (to compute how much X-coupling should be compensated for). */
	private AbsoluteDate lastOopManeuverDate;

    /** OOP X-coupling pre-compensation: margin to avoid driving the eccentricity off-circle. */
	private final double oopPrecompensationMargin = .8;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model in-plane maneuver model
     * @param yawFlipSequence array of pairs containing a day-of-year and the corresponding new status of the yaw flip (0 or 1)
     * @param centerX abscissa of the circle center
     * @param centerY ordinate of the circle center
     * @param meanRadius radius of the controlled circle
     * @param maxRadius radius of the violation circle
     * @param singleBurn flag for single burn recognition
     * @param sun Sun model
     * @param samplingStep step to use for sampling throughout propagation
     * @param horizon time horizon duration
     * @param oopManeuver optional OOP maneuver which X-coupling will be compensated for (null if dummy)
     */
    public EccentricityCircle(final String name, final String controlledName, final int controlledIndex,
                              final TunableManeuver[] model, int[][] yawFlipSequence, final double centerX, final double centerY,
                              final double meanRadius, final double maxRadius, final boolean singleBurn,
                              final CelestialBody sun, final double samplingStep, final double horizon, TunableManeuver oopManeuver) {
        super(name, model, yawFlipSequence, controlledName, controlledIndex, null, -1, 0, maxRadius, horizon * Constants.JULIAN_DAY);
        this.stephandler  = new Handler();
        this.centerX      = centerX;
        this.centerY      = centerY;
        this.meanRadius   = meanRadius;
        this.singleBurn   = singleBurn;
        this.sun          = sun;
        this.samplingStep = samplingStep;
        this.oopManeuver  = oopManeuver;
        

        xModel = new SecularAndHarmonic(0,
                                         new double[] {
                                             SUN_PULSATION,
                                             EARTH_PULSATION,
                                             MOON_PULSATION,
                                             EARTH_PULSATION - 2 * MOON_PULSATION,
                                             EARTH_PULSATION - 2 * SUN_PULSATION
                                         });
        yModel = new SecularAndHarmonic(0,
                                         new double[] {
                                             SUN_PULSATION,
                                             EARTH_PULSATION,
                                             MOON_PULSATION,
                                             EARTH_PULSATION - 2 * MOON_PULSATION,
                                             EARTH_PULSATION - 2 * SUN_PULSATION
                                         });

        // rough order of magnitudes values for initialization purposes
        xModel.resetFitting(AbsoluteDate.J2000_EPOCH,
                             new double[] {
                                 centerX, 1.0e-2, 1.0e-2, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5
                             });
        yModel.resetFitting(AbsoluteDate.J2000_EPOCH,
                             new double[] {
                                 centerY, 1.0e-2, 1.0e-2, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5, 1.0e-5
                             });

    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end)
        throws OrekitException {

        resetMarginsChecks();

        // select a long maneuver-free interval for fitting
        final AbsoluteDate[] freeInterval = getManeuverFreeInterval(maneuvers, fixedManeuvers, start, end);
        
        // Search for new OOP maneuvers performed by any other control law (same control loop)
        //updateLastOopManeuverDate(maneuvers);

        // Search for new OOP maneuvers performed by any other control loop
        updateLastOopManeuverDate(fixedManeuvers);

        fitStart       = propagator.propagate(freeInterval[0]);
        cycleStart     = start;
        timeHorizonEnd = end;

        if (iteration == 0) {
            // reconstruct eccentricity motion model only on first iteration

            // fit short long periods model to eccentricity
            xModel.resetFitting(freeInterval[0], xModel.getFittedParameters());
            yModel.resetFitting(freeInterval[0], yModel.getFittedParameters());
            for (AbsoluteDate date = freeInterval[0]; date.compareTo(freeInterval[1]) < 0; date = date.shiftedBy(samplingStep)) {
                final EquinoctialOrbit orbit =
                        (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(propagator.propagate(date).getOrbit());
                xModel.addPoint(date, orbit.getEquinoctialEx());
                yModel.addPoint(date, orbit.getEquinoctialEy());
            }

            xModel.fit();
            yModel.fit();
        }

    }

    private void updateLastOopManeuverDate(ScheduledManeuver[] maneuvers) {
    	
		// create a list and copy the array content into it
    	final List<ScheduledManeuver> maneuversList = new ArrayList<ScheduledManeuver>();
    	
        for (final ScheduledManeuver maneuver : maneuvers) {
            	// copy the maneuvers array into the list
        		maneuversList.add(maneuver);
            }

        // Call the original function (accepting only a list of maneuvers as input)
        updateLastOopManeuverDate(maneuversList);
	}

	private void updateLastOopManeuverDate(List<ScheduledManeuver> fixedManeuvers) {
    	
    	// Do something only if there is some OOP maneuver to monitor and compensate for
    	if(oopManeuver != null){
    		
    		if(lastOopManeuverDate == null){
	    		// If no OOP maneuver has been performed so far, initialize search date at simulation start
    			lastOopManeuverDate = oopManeuver.getBeginningDate();
    		}
	        
	        // Loop on new OOP maneuvers and remember the last one
	        for (final ScheduledManeuver maneuver : fixedManeuvers) {
	            final AbsoluteDate date = maneuver.getDate();
	            if (maneuver.getName().equals(oopManeuver.getName()) && date.compareTo(lastOopManeuverDate) >= 0) {
	                lastOopManeuverDate = date;
	            }
	        }
    	}
	}

	/** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException {

        // Check which maneuver tuning is needed
        if (singleBurn && getMargins() >= 0) {
        	
        	// Single burn case, within margins
        	// Change only the point of application of the longitude maneuvers
        	// The delta-e magnitude remains the one needed for longitude but 
        	// the direction is enhanced
        	
            
            // try to select a pair of in-plane maneuvers
            final int[] indices = selectManeuversPair(tunables);

            if ((indices[1] < 0) || (indices[0] >= 0 && indices[1] >= 0)) {

                // we have zero maneuver or a full pair (already eccentricity-optimized)
            	// do nothing
                return tunables;
 
                
            } else {
            	// indices[0] < 0 && indices [1] >= 0
            	// we have one isolated IP maneuver (longitude)
            	// tune its date to correctly orient its delta-eccentricity
            	final int index = indices[1];
            	            	
                // mean eccentricity at cycle middle time
                final AbsoluteDate middleDate = cycleStart.shiftedBy(0.5 * getCycleDuration());
                final double meanEx = xModel.meanValue(middleDate, 0, 1);
                final double meanEy = yModel.meanValue(middleDate, 0, 1);

                // desired eccentricity at cycle middle time
                final PVCoordinates sunPV = sun.getPVCoordinates(middleDate, fitStart.getFrame());
                final double alphaSun = sunPV.getPosition().getAlpha();
                final double targetEx = centerX + meanRadius * FastMath.cos(alphaSun);
                final double targetEy = centerY + meanRadius * FastMath.sin(alphaSun);
                
                // Approximate OOP X-coupling perturbation on the eccentricity
                final Vector2D deOopCrossCoupling = getOopCrossCouplingPerturbation();

                // eccentricity change needed
                double deX = targetEx - meanEx;
                double deY = targetEy - meanEy;
                
                // OOP precompensation case
                if(oopManeuver != null){
    	            if(FastMath.hypot(targetEx - deOopCrossCoupling.getX(), targetEy - deOopCrossCoupling.getY()) < getMax()*oopPrecompensationMargin){
    	            	// add OOP compensation
    	            	deX -= deOopCrossCoupling.getX();
    	            	deY -= deOopCrossCoupling.getY();
    	            }else{
    	            	// If it would drive us off circle, just maintain the eccentricity as is
    	            	deX = 0;
    	            	deY = 0;
    	            }
                }

                final ScheduledManeuver[] tuned;
                final AdapterPropagator adapterPropagator = new AdapterPropagator(reference);

                // copy the array
            	tuned = tunables.clone();
                        
            	// the maneuver magnitude won't be changed 
            	final double dV0 = tunables[index].getSignedDeltaV();

            	// compute the desired application angle of the maneuver
                final SpacecraftState state  = tunables[index].getStateBefore();
                final double sign            = thrustSignVelocity(state,getModel());
                final EquinoctialOrbit orbit = (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state.getOrbit());
                final double alphaState      = orbit.getLM();
                final double alphaMan        = FastMath.atan2(sign * deY, sign * deX);
                final double dAlpha          = MathUtils.normalizeAngle(alphaMan - alphaState, 0);
                final double n               = state.getKeplerianMeanMotion();

                // derive the corresponding application date
                AbsoluteDate t = state.getDate().shiftedBy(dAlpha / n);
                final AbsoluteDate t0  = (t.compareTo(cycleStart.getDate()) <= 0) ? t.shiftedBy(2 * FastMath.PI / n) : t;
                                
                // adjust maneuver
                final TunableManeuver model = getModel();
                final ScheduledManeuver m0 = new ScheduledManeuver(model, t0, new Vector3D(dV0, model.getDirection()),
                                                                   model.getCurrentThrust(), model.getCurrentISP(),
                                                                   adapterPropagator, false);
                adapterPropagator.addEffect(new SmallManeuverAnalyticalModel(m0.getStateBefore(),
                                                                             m0.getDeltaV(),
                                                                             m0.getIsp()));
                tuned[index] = m0;


                // change the trajectory of untouched maneuvers
                for (int i = 0; i < tuned.length; ++i) {
                    if (i != index) {
                        final ScheduledManeuver maneuver =
                                new ScheduledManeuver(tuned[i].getModel(), tuned[i].getDate(),
                                                      tuned[i].getDeltaV(), tuned[i].getThrust(),
                                                      tuned[i].getIsp(), adapterPropagator,
                                                      tuned[i].isReplanned());
                        tuned[i] = maneuver;
                        adapterPropagator.addEffect(new SmallManeuverAnalyticalModel(maneuver.getStateBefore(),
                                                                                     maneuver.getDeltaV(),
                                                                                     maneuver.getIsp()));
                    }
                }

                return tuned;

            }

            
        } else {
        	// Double burn case (or single burn, but out of margins)
        	// Re-tune previous IP maneuvers, split them or create new specifically 
        	// eccentricity-related maneuver pair
        	
            // mean eccentricity at cycle middle time
            final AbsoluteDate middleDate = cycleStart.shiftedBy(0.5 * getCycleDuration());
            final double meanEx = xModel.meanValue(middleDate, 0, 1);
            final double meanEy = yModel.meanValue(middleDate, 0, 1);

            // desired eccentricity at cycle middle time
            final PVCoordinates sunPV = sun.getPVCoordinates(middleDate, fitStart.getFrame());
            final double alphaSun = sunPV.getPosition().getAlpha();
            final double targetEx = centerX + meanRadius * FastMath.cos(alphaSun);
            final double targetEy = centerY + meanRadius * FastMath.sin(alphaSun);
            

            // eccentricity change needed
            double deX = targetEx - meanEx;
            double deY = targetEy - meanEy;
            
            // OOP pre-compensation case, within margins
            if(oopManeuver != null && getMargins() >= 0){
            	
	            // Approximate OOP X-coupling perturbation on the eccentricity
	            final Vector2D deOopCrossCoupling = getOopCrossCouplingPerturbation();
	            
	            if(FastMath.hypot(targetEx - deOopCrossCoupling.getX(), targetEy - deOopCrossCoupling.getY()) < getMax()*oopPrecompensationMargin){
	            	// add OOP compensation, except if it brings the eccentricity too close to the margin
	            	deX -= deOopCrossCoupling.getX();
	            	deY -= deOopCrossCoupling.getY();
	            }else{
	            	// If it would drive us too close, just maintain the eccentricity as is
	            	deX = 0;
	            	deY = 0;
	            }
            }
            
            final double vs  = fitStart.getPVCoordinates().getVelocity().getNorm();
            final double dV  = 0.5 * FastMath.hypot(deX, deY) * vs;

            final ScheduledManeuver[] tuned;
            final AdapterPropagator adapterPropagator = new AdapterPropagator(reference);

            // try to select two in-plane maneuvers we can adjust
            final int[] indices = selectManeuversPair(tunables);

            // adjust the maneuvers (or create them)
            final AbsoluteDate t0;
            double dV0;
            final AbsoluteDate t1;
            double dV1;
            if (indices[1] < 0) {

                // we don't have even one maneuver, create two maneuvers from scratch
                tuned = new ScheduledManeuver[tunables.length + 2];
                System.arraycopy(tunables, 0, tuned, 0, tunables.length);
                indices[0] = tunables.length;
                indices[1] = tunables.length + 1;

                final EquinoctialOrbit orbit =
                        (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(fitStart.getOrbit());
                final double alphaStart = orbit.getLM();
                final double alphaMan   = FastMath.atan2(deY, deX);
                final double dAlpha     = MathUtils.normalizeAngle(alphaMan - alphaStart, 2 * FastMath.PI);
                final double n          = fitStart.getKeplerianMeanMotion();

                t0  = fitStart.getDate().shiftedBy(dAlpha / n);
                dV0 = +0.5 * dV;
                t1  = t0.shiftedBy(FastMath.PI / n);
                dV1 = -0.5 * dV;

            } else {

                final double dVExisting;
                if (indices[0] < 0) {

                    // we have one maneuver to split
                    dVExisting = tunables[indices[1]].getSignedDeltaV();

                    // add one element to the array
                    tuned = new ScheduledManeuver[tunables.length + 1];
                    System.arraycopy(tunables, 0, tuned, 0, tunables.length);
                    indices[0] = indices[1];
                    indices[1] = tunables.length;

                } else {

                    // we have two existing maneuvers to adjust
                    dVExisting = tunables[indices[0]].getSignedDeltaV() +
                                 tunables[indices[1]].getSignedDeltaV();

                    // copy the array
                    tuned = tunables.clone();

                }

                final SpacecraftState state  = tunables[indices[0]].getStateBefore();
                final double sign            = thrustSignVelocity(state, getModel());
                final EquinoctialOrbit orbit = (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state.getOrbit());
                final double alphaState      = orbit.getLM();
                final double alphaMan        = FastMath.atan2(sign * deY, sign * deX);
                final double dAlpha          = MathUtils.normalizeAngle(alphaMan - alphaState, 0);
                final double n               = state.getKeplerianMeanMotion();

                // adjust the maneuver(s)
                AbsoluteDate t = state.getDate().shiftedBy(dAlpha / n);
                t0 = (t.compareTo(cycleStart.getDate()) <= 0) ? t.shiftedBy(2 * FastMath.PI / n) : t;
                dV0 = 0.5 * (dVExisting + sign * dV);
                t1  = t0.shiftedBy(FastMath.PI / n);
                dV1 = 0.5 * (dVExisting - sign * dV);

            }

            // take the limits into account, trying to preserve the sum dV0 + dV1 if possible
            double[] trimmed = trimManeuvers(dV0, dV1);

            // adjust the two selected maneuvers
            final TunableManeuver model = getModel();
            final ScheduledManeuver m0 = new ScheduledManeuver(model, t0, new Vector3D(trimmed[0], model.getDirection()),
                                                               model.getCurrentThrust(), model.getCurrentISP(),
                                                               adapterPropagator, false);
            adapterPropagator.addEffect(new SmallManeuverAnalyticalModel(m0.getStateBefore(),
                                                                         m0.getDeltaV(),
                                                                         m0.getIsp()));
            tuned[indices[0]] = m0;

            final ScheduledManeuver m1 = new ScheduledManeuver(model, t1, new Vector3D(trimmed[1], model.getDirection()),
                                                               model.getCurrentThrust(), model.getCurrentISP(),
                                                               adapterPropagator, false);
            adapterPropagator.addEffect(new SmallManeuverAnalyticalModel(m1.getStateBefore(),
                                                                         m1.getDeltaV(),
                                                                         m1.getIsp()));
            tuned[indices[1]] = m1;


            // change the trajectory of untouched maneuvers
            for (int i = 0; i < tuned.length; ++i) {
                if (i != indices[0] && i != indices[1]) {
                    final ScheduledManeuver maneuver =
                            new ScheduledManeuver(tuned[i].getModel(), tuned[i].getDate(),
                                                  tuned[i].getDeltaV(), tuned[i].getThrust(),
                                                  tuned[i].getIsp(), adapterPropagator,
                                                  tuned[i].isReplanned());
                    tuned[i] = maneuver;
                    adapterPropagator.addEffect(new SmallManeuverAnalyticalModel(maneuver.getStateBefore(),
                                                                                 maneuver.getDeltaV(),
                                                                                 maneuver.getIsp()));
                }
            }

            return tuned;
        }

    }

    
	/** Compute an approximated value of the eccentricity drift due to OOP X-coupling.
     * <p>
     * We want to compensate for past and future eccentricity bumps due to 
     * OOP maneuvers X-coupling. This function approximates the average 
     * delta-V spent for OOP control, multiplies it by the X-coupling set by
     * the user and returns the average eccentricity drift that eccentricity
     * control maneuvers will have to compensate for.
     * </p>
     * @return eccentricity perturbation since last OOP maneuver
     */
    private Vector2D getOopCrossCouplingPerturbation() throws OrekitException {
    	
    	if(oopManeuver == null){
    		return new Vector2D(0., 0.);
    		
    	}else{
    		// When no OOP maneuver has been performed yet, consider the start of the simulation as the last one
	    	AbsoluteDate lastManeuverDate = (lastOopManeuverDate == null) ? oopManeuver.getBeginningDate() : lastOopManeuverDate.getDate();
    			
			// Delta-V probably spent if the next inclination maneuver would happen now
	    	// This is the average OOP delta-V per time (~45m/s/year) integrated over the time since last OOP maneuver
			final double meanOopDvPerCycle = MEAN_OOP_DV_PER_YEAR / Constants.JULIAN_YEAR * cycleStart.durationFrom(lastManeuverDate);

			// Inertial velocity
			final double velocity = fitStart.getPVCoordinates(FramesFactory.getMOD(false)).getVelocity().getNorm();

			// Assuming that the OOP maneuvers are performed as single North burns,
			// derive the probable eccentricity perturbation of next OOP maneuver:
			// OOP X-coupling along +X moves the eccentricity towards -Y
			// OOP X-coupling along +Z moves the eccentricity towards -X
			final double deX = - meanOopDvPerCycle / velocity * oopManeuver.getDirection().getZ();
			final double deY = - meanOopDvPerCycle / velocity * oopManeuver.getDirection().getX();
			return new Vector2D(deX, deY);
    	}
	}

	/** Ensure a maneuvers pair to fulfill constraints.
     * <p>
     * We have an initial pair of maneuvers (dV0, dV1) attempting to control
     * eccentricity. This pair must fulfill the dVinf and dVsup limits at
     * all costs. As the sum dV0 + dV1 is inherited from earlier control laws,
     * typically longitude control, it must also be preserved if possible. In
     * fact, this should always be possible since the previous control laws
     * should have ensured their maneuvers already fulfill the constraints and
     * this control law only change the dV distribution, not the overall sum.
     * </p>
     * @param dV0 first maneuver
     * @param dV1 second maneuver
     * @return trimmed maneuvers
     */
    private double[] trimManeuvers(final double dV0, final double dV1) {

        // ensure the sum fulfills the constraints (this should really be a no-op)
        final double inf = getModel().getCurrentDVInf();
        final double sup = getModel().getCurrentDVSup();
        final double sum = FastMath.max(2 * inf, FastMath.min(2 * sup, dV0 + dV1));
        final double sumCorrection = 0.5 * (sum - (dV0 + dV1));
        final double dVA = dV0 + sumCorrection;
        final double dVB = dV1 + sumCorrection;

        if ((dVA >= inf) && (dVA <= sup) && (dVB >= inf) && (dVB <= sup)) {
            // nothing to fix
            return new double[] { dVA, dVB };
        }

        // find the active limit (either inf or sup) corresponding to the
        // intersection points of the line corresponding to dVA + dVB = sum
        // with the limit rectangle in the (dVA, dVB) plane
        final double activeLimit = (sum >= inf + sup) ? sup : inf;

        // the remaining dV is within limits by construction
        final double otherDV = sum - activeLimit;

        // select the pair corresponding to the smallest change
        if (FastMath.hypot(dVA - activeLimit, dVB - otherDV) <=
            FastMath.hypot(dVA - otherDV, dVB - activeLimit)) {
            return new double[] { activeLimit, otherDV };
        } else {
            return new double[] { otherDV, activeLimit };
        }
            
    }

    /** Select a pair of in-plane maneuvers roughly on oppiste sides of orbit.
     * @param maneuvers available maneuvers to select from
     * @return two indices in the array (set to -1 if a maneuver is missing)
     * @exception PropagationException if maneuver state cannot be computed
     */
    private int[] selectManeuversPair(final ScheduledManeuver[] maneuvers)
        throws PropagationException {

        final int[] indices = new int[2];
        indices[0] = -1;
        indices[1] = -1;

        final double period = fitStart.getKeplerianPeriod();

        AbsoluteDate date1 = null;
        for (int i = maneuvers.length - 1; i >= 0; --i) {
            if (maneuvers[i].getName().equals(getModel().getName())) {
                if (indices[1] < 0) {
                    // first maneuver to be found
                    indices[1] = i;
                    date1      = maneuvers[i].getDate();
                } else {
                    final AbsoluteDate date0 = maneuvers[i].getDate();
                    final double nbHalfOrbits = 2 * date1.durationFrom(date0) / period;
                    if ((((int) FastMath.rint(nbHalfOrbits)) % 2) == 1) {
                        // this maneuver is on the other side of the orbit as the previous one,
                        // select it
                        indices[0] = i;
                        return indices;
                    } else {
                        // this maneuver is on the same side of the orbit as the previous one,
                        // discard the previous one
                        indices[1] = i;
                        date1      = date0;
                    }
                }
            }
        }

        return indices;

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
        private static final long serialVersionUID = 220407581859026265L;

        /** {@inheritDoc} */
        public void init(final SpacecraftState s0, final AbsoluteDate t) {
//            resetMarginsChecks();
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

                    if (date.compareTo(fitStart.getDate()) >= 0) {
                        
                        // Check limits only during the current cycle
                        if (date.durationFrom(cycleStart) <= EccentricityCircle.this.getCycleDuration() ) {
                            // compute current eccentricity
                            interpolator.setInterpolatedDate(date);
                            final SpacecraftState state  = interpolator.getInterpolatedState();
                            final EquinoctialOrbit orbit = (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state.getOrbit());

                            // check limits
                            checkMargins(date,
                                         FastMath.hypot(orbit.getEquinoctialEx() - centerX,
                                                        orbit.getEquinoctialEy() - centerY));
                        }

                    } else {
                        // if step is too short, assume limits are not violated
                        checkMargins(date, 0.0);
                    }

                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }

        }

    }

}
