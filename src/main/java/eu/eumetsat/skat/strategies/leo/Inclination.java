/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BracketingNthOrderBrentSolver;
import org.apache.commons.math3.optimization.GoalType;
import org.apache.commons.math3.optimization.univariate.BrentOptimizer;
import org.apache.commons.math3.optimization.univariate.UnivariateOptimizer;
import org.apache.commons.math3.optimization.univariate.UnivariatePointValuePair;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.DateTimeComponents;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Station-keeping control for inclination in sun synchronous Low Earth Orbits.
 * <p>
 * TBW
 * </p>
 * 
 * @author Daniel Aguilar
 */
public class Inclination extends AbstractLeoSKControl {

    /** Indicator for compensating long burns inefficiency. */
    private final boolean compensateLongBurn;

    /** Reference state at first node in eclipse. */
    private SpacecraftState nodeState;

    /** Reference date for analytical models. */
    private AbsoluteDate t0;

    /** Initial Mean Local Solar Time for models. */
    private double inc0;

    /** Reference inclination offset */
    private double iOffsetRef;

    /** Propagator. */
    private Propagator propagator;

    /** Maneuvers Day Of Year. */
    private final int[] maneuversDoy;

    /** Analytical models for Inclination. */
    private final InclinationModel inclinationModel;

    /** Targeting function. */
    private UnivariateFunction targeting;

    /** Interface for providing Mean Local Solar Time analytical model. */
    public interface InclinationModel {

        /** Check if inclination increases or decreases with time.
         * @return true if inclination increases or decreases with time
         */
        boolean increasingInclination();

        /** Compute Inclination.
         * @param t current date
         * @param t0 reference date
         * @param inc0 reference inclination at t0
         * @return Inclination at {@code t}
         */
        double inc(AbsoluteDate t, AbsoluteDate t0, double inc0, final double iOffsetRef);

    }

    /**
     * Simple constructor.
     * 
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model out-of-plane maneuver model
     * @param maneuverSequence 
     * @param firstOffset time offset of the first maneuver with respect to cycle start
     * @param maxManeuvers maximum number of maneuvers to set up in one cycle
     * @param orbitsSeparation minimum time between split parts in number of orbits
     * @param earth Earth model
     * @param sun Sun model
     * @param referenceRadius reference radius of the Earth for the potential model (m)
     * @param mu central attraction coefficient (m<sup>3</sup>/s<sup>2</sup>)
     * @param j2 un-normalized zonal coefficient (about +1.08e-3 for Earth)
     * @param solarTime target solar time (in fractional hour, i.e 9h30 = 9.5)
     * @param solarTimetolerance solar time tolerance (in hours)
     * @param horizon time horizon duration
     * @param compensateLongBurn if true, long burn inefficiency should be compensated
     * @param phasingDays number of days of the phasing cycle
     * @param phasingOrbits number of orbits of the phasing cycle
     * @param maneuversDoy maneuvers Day Of Year
     * @param analyticalModels analytical models for inclination offset and MLST
     * @exception OrekitException if the UTC-TAI correction cannot be loaded
     */
    public Inclination(final String name, final String controlledName, final int controlledIndex,
                              final TunableManeuver[] model, int[][] maneuverSequence, final double firstOffset, final int maxManeuvers,
                              final int orbitsSeparation, final OneAxisEllipsoid earth, final CelestialBody sun,
                              final double referenceRadius, final double mu, final double j2,
                              final double incMeanValue, final double incDeadband, final double horizon,
                              final boolean compensateLongBurn, final int phasingDays, final int phasingOrbits,
                              final int[] maneuversDoy, final InclinationModel analyticalModels)
        throws OrekitException {

        super(name, controlledName, controlledIndex, model,
              maneuverSequence, firstOffset, maxManeuvers, orbitsSeparation, earth, sun, referenceRadius, mu, j2,
              incMeanValue - incDeadband, incMeanValue + incDeadband, horizon * Constants.JULIAN_DAY);

        this.compensateLongBurn = compensateLongBurn;
        this.maneuversDoy = maneuversDoy.clone();
        Arrays.sort(this.maneuversDoy);
        this.inclinationModel = analyticalModels;
        meanPeriod        = phasingDays * Constants.JULIAN_DAY / phasingOrbits;

    }

    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final int cycle,
                              final ScheduledManeuver[] maneuvers,
                              final Propagator propagator,
                              final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start,
                              final AbsoluteDate end) throws OrekitException, SkatException {

        resetMarginsChecks();
        this.iteration  = iteration;
        this.cycleStart = start;
        this.cycleEnd   = end;
        this.propagator = propagator;

        if (iteration == 0) {
            nodeState = null;
        }

        // select a long maneuver-free interval for fitting
        freeInterval = getManeuverFreeInterval(maneuvers, fixedManeuvers, start, end);
        double stepSize = meanPeriod / 100.0;

        // find all crossings
        List<SpacecraftState> crossings = new ArrayList<SpacecraftState>();
        SpacecraftState crossing = firstLatitudeCrossing(0.0, true, earth,
                                                         start.shiftedBy(meanPeriod), end, stepSize, propagator);
        checkMargins(crossing.getDate(), inclinationMOD(crossing));
        if (crossing.getDate().compareTo(freeInterval[0]) >= 0 &&
            crossing.getDate().compareTo(freeInterval[1]) <= 0) {
            crossings.add(crossing);
        }

        // find all other latitude crossings from regular schedule
        double deltaT = meanPeriod;
        while (crossing != null && crossing.getDate().shiftedBy(deltaT).compareTo(end.shiftedBy(-meanPeriod)) < 0) {

            final AbsoluteDate previous = crossing.getDate();
            crossing = latitudeCrossing(0.0, earth, previous.shiftedBy(deltaT), end, stepSize, meanPeriod / 8, propagator);
            if (crossing != null) {
                checkMargins(crossing.getDate(), inclinationMOD(crossing));
                if (crossing.getDate().compareTo(freeInterval[0]) >= 0 &&
                    crossing.getDate().compareTo(freeInterval[1]) <= 0) {
                    crossings.add(crossing);
                }
                deltaT = crossing.getDate().durationFrom(previous);
            }

        }

        // set up reference date for analytical models
        if (! crossings.isEmpty()) {
        	crossing = crossings.get(0);
        	t0 = crossing.getDate();
            inc0 = inclinationMOD(crossings.get(0));
        } else {
            t0    = null;
            inc0  = Double.NEGATIVE_INFINITY;
        }

        // set inclination offset
        iOffsetRef = 0;

    }

    /** Compute the inclination in MOD.
     * 
     *  @param state current spacecraft state
     *  @return inclination in MOD
     *  @exception OrekitException if state cannot be converted
     */
    private double inclinationMOD(final SpacecraftState state) throws OrekitException {
    	final CartesianOrbit modOrbit = new CartesianOrbit(state.getPVCoordinates(FramesFactory.getMOD(false)),
    			FramesFactory.getMOD(false),state.getDate(),state.getMu()); 
    	final SpacecraftState scState = new SpacecraftState(modOrbit, state.getMass());
    	return scState.getI();
    	//final OsculatingToMeanElementsConverter o2m = new OsculatingToMeanElementsConverter(scState,1,propagator);
    	//return o2m.getMeanInclination();
    }

    /** Find a min/max inclination value.
     * @param tMin search start
     * @param tMax search end
     * @param searchMin if true, search a min value, otherwise search for a max value
     * @return extrememum (with reference date at tMin)
     */
    private UnivariatePointValuePair extremumInclination(final AbsoluteDate tMin, final AbsoluteDate tMax,
                                                         final boolean searchMin) {

        final UnivariateFunction inc = new UnivariateFunction() {
            /** {@inheritDoc} */
            public double value(double x) {
                return inclinationModel.inc(tMin.shiftedBy(x), t0, inc0, iOffsetRef);
            }
        };

        final UnivariateOptimizer optimizer = new BrentOptimizer(1.0e-10, 1.0);

        return optimizer.optimize(1000, inc, searchMin ? GoalType.MINIMIZE : GoalType.MAXIMIZE,
                                  0, tMax.durationFrom(tMin));

    }

    /** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference)
        throws OrekitException, SkatException {

    	// get maneuver date index inside interval
    	final int manoDoyIndex = getManeuverIndex(cycleStart,cycleEnd);
    	
    	// if no maneuver inside this interval
    	if(manoDoyIndex<0) {
    		//do nothing
    		return tunables;
    	}

    	// get maneuver DoY (doy1) and next maneuver DoY (doy2)
    	int doy1 = maneuversDoy[manoDoyIndex];
    	int doy2 = maneuversDoy[(manoDoyIndex+1) % maneuversDoy.length];
    	
        // find next maneuvers opportunities
        final DateTimeComponents dtc =
                cycleStart.shiftedBy(firstOffset).getComponents(TimeScalesFactory.getUTC());
        final int startDoy = dtc.getDate().getDayOfYear();

        // update maneuvers DoY
        while (doy1 <= startDoy) {
            doy1 += 365;
        }
        while (doy2 <= doy1) {
            doy2 += 365;
        }
        AbsoluteDate firstOpportunity  =
                cycleStart.shiftedBy(firstOffset + (doy1 - startDoy) * Constants.JULIAN_DAY);
        AbsoluteDate secondOpportunity =
                firstOpportunity.shiftedBy((doy2 - doy1) * Constants.JULIAN_DAY);

        // check MLST evolution on a very long period (typically much longer than cycle)
        final AbsoluteDate start;
        final AbsoluteDate targetT;
        if (nodeState == null) {
            start   = cycleStart;
        } else {
            start   = nodeState.getDate();
        }
        targetT = start.shiftedBy(2*Constants.JULIAN_YEAR);

        	
        if (iteration == 0) {

            // look at current excursion (achieved peak, and following tail)
            final UnivariatePointValuePair pvPeak =
                    extremumInclination(start, targetT, inclinationModel.increasingInclination());
            final AbsoluteDate peakDate = start.shiftedBy(pvPeak.getPoint());
            final UnivariatePointValuePair pvTail =
                    extremumInclination(peakDate, targetT, !inclinationModel.increasingInclination());

            // get exit date
            final double incExit = getMin();
            final UnivariateFunction exitFunction = new UnivariateFunction() {
            	/** {@inheritDoc} */
            	public double value(double x) {
            		return inclinationModel.inc(peakDate.shiftedBy(x), t0, inc0, iOffsetRef) -
                          incExit;
            	}
            };

            // get exit date
            final AbsoluteDate exitDate;
            if (pvPeak.getValue() > getMax()) {
            	// if peak value is above maximum value
        		// do nothing
        		return tunables;           	
            } else if(pvPeak.getValue() < getMin()) {
            	// if peak value is below minimum value
                exitDate = start;
            } else if (pvTail.getValue() > getMin()) {
            	// if tail value is above minimum value
                exitDate = targetT.shiftedBy(1.0);
            } else {
            	// otherwise
                final double tExit = new BracketingNthOrderBrentSolver(1.0e-10, 5).solve(1000, exitFunction,
                        0, pvTail.getPoint());
                exitDate = peakDate.shiftedBy(tExit);
            }

            // if exit date is after next maneuver opportunity
            if (exitDate.compareTo(secondOpportunity)>0) {
            	
                // no maneuver needed
                nodeState = null;

            } else {
                // otherwise (i.e. if exit date is before next maneuver opportunity)
                // maneuver is needed

                // get node state (used for maneuver date)
                if (firstOpportunity.durationFrom(cycleEnd) < -2.5*meanPeriod) {
                    // maneuver opportunity is well inside the cycle, we can find a node close to it
                    nodeState = findManeuverNode(firstOpportunity, cycleEnd.shiftedBy(-1.5 * meanPeriod), propagator);
                } else {
                    // maneuver opportunity is too close to cycle end, we need some margin to find the node
                    nodeState = findManeuverNode(cycleEnd.shiftedBy(-4.5 * meanPeriod), cycleEnd.shiftedBy(-1.5 * meanPeriod), propagator);
                }
            }

            
            // target a centered excursion for the observing period
            targeting = new UnivariateFunction() {
                /** {@inheritDoc} */
                public double value(double x) {
                    final double savedIOffsetRef = iOffsetRef;
                    iOffsetRef += x;
                    final UnivariatePointValuePair pvPeak =
                            extremumInclination(nodeState.getDate(), targetT, inclinationModel.increasingInclination());
                    iOffsetRef = savedIOffsetRef;
                    return (pvPeak.getValue() - getMax());
                }
            };
        }

        if (nodeState == null) {
            // no maneuvers needed
            return tunables;
        }

        // compute inclination offset needed to achieve station-keeping target
        double deltaOffset = findZero(targeting, 0, -0.1, 0.1, 1.0e-6, 0.1, 1.0e-10);
        if (Double.isNaN(deltaOffset)) {
            throw new SkatException(SkatMessages.NO_BRACKETING);
        }

        // check for overshoot
        final double savedIOffsetRef = iOffsetRef;
        iOffsetRef += deltaOffset;
        final UnivariatePointValuePair pvPeak =
                extremumInclination(nodeState.getDate(), targetT, inclinationModel.increasingInclination());
        iOffsetRef = savedIOffsetRef;
//        if (pvPeak.getValue() > getMax() || pvPeak.getValue() <= getMin()) {
        if (pvPeak.getValue() > getMax()) {
//            final double safetyMargin = 0.1;
//            final double desiredPeak  = pvPeak.getValue() > getMax() ?
//                                        ((1 - safetyMargin) * getMax() + safetyMargin * getMin()) :
//                                        (safetyMargin * getMax() + (1 - safetyMargin) * getMin());
            final double desiredPeak = getMax();
            deltaOffset = findZero(new UnivariateFunction() {
                /** {@inheritDoc} */
                public double value(double x) {
                    final UnivariatePointValuePair p = extremumInclination(nodeState.getDate(), targetT, inclinationModel.increasingInclination());
                    return p.getValue() - desiredPeak;
                }
            }, deltaOffset,  deltaOffset - 0.1, deltaOffset + 0.1, 1.0e-6, 0.1, 1.0e-10);
            if (Double.isNaN(deltaOffset)) {
                throw new SkatException(SkatMessages.NO_BRACKETING);
            }
        }

        return tuneInclinationManeuver(tunables, reference, nodeState, deltaOffset, compensateLongBurn);

    }

    /** TBW
     * @param TVW
     * @param TBW
     * @return TBW
     * @throws OrekitException 
     */
    private int getManeuverIndex(final AbsoluteDate start, final AbsoluteDate end) throws OrekitException {

    	// check maneuver size
    	if(maneuversDoy.length>=1) {
    		
        	//check maneuver one by one
            for (int i = 0; i < maneuversDoy.length; ++i) {
            	
            	if(isManeuverInsideInterval(i,start,end)) {
            		return i;
            	} else if(i==maneuversDoy.length-1) {
            		return -1;
            	}
            	
            }

    	}
    	
    	return -1;
    }
    
    /** TBW
     * @param TVW
     * @param TBW
     * @param TBW
     * @return TBW
     * @throws OrekitException 
     */
    private boolean isManeuverInsideInterval(final int manoIdx, final AbsoluteDate start, final AbsoluteDate end) throws OrekitException {

    	if ( (manoIdx>=0) && (manoIdx<maneuversDoy.length) ) {
    		
    		final int startDoY = start.getComponents(TimeScalesFactory.getUTC()).getDate().getDayOfYear();
    		final int endDoY   = end.getComponents(TimeScalesFactory.getUTC()).getDate().getDayOfYear();
    		
    		if (startDoY<=endDoY) {
    			
    			return (maneuversDoy[manoIdx]>=startDoY) && (maneuversDoy[manoIdx]<=endDoY);
    			    			
    		} else {
    			
    			return !(maneuversDoy[manoIdx]>=endDoY) && (maneuversDoy[manoIdx]<=startDoY);
    			
    		}

    	}
    	
    	return false;
    }

    /** {@inheritDoc} */
    public EventDetector getEventDetector() {
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return null;
    }

}

