/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.analysis.UnivariateFunction;
import org.apache.commons.math.analysis.solvers.BracketingNthOrderBrentSolver;
import org.apache.commons.math.analysis.solvers.BrentSolver;
import org.apache.commons.math.analysis.solvers.UnivariateRealSolverUtils;
import org.apache.commons.math.exception.NoBracketingException;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.stat.descriptive.rank.Percentile;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.AbstractDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;

/**
 * Station-keeping control attempting to get mean local solar time in a deadband.
 * The deadband is located around a main value {@link #center}. A tolerance margin
 * can be define through the minSolarTime and maxSolarTime parameters, defined by
 * construction. If a violation occurs by getting out of the [minSolarTime, maxSolarTime] 
 * interval, a notifier will be triggered and this information will be monitored.
 *  <p>
 * This control value is:
 * <pre> max(|MLST<sub>75</sub> - MLST<sub>c</sub>|,|MLST<sub>c</sub> - MLST<sub>25</sub>|)</pre>
 * where MLST<sub>75</sub> and MLST<sub>25</sub> are the spacecraft mean local solar time 1st and 3rd
 * quartiles evaluated for the complete cycle duration and MLST<sub>c</sub> is the target mean local
 * solar time.
 * </p>
 * <p>
 * The previous definition implies that setting the target of this control
 * to MLST<sub>c</sub> attempts to have most of the points mean local solar time
 * for the satellite centered around the target mean local solar time during the
 * station-keeping. 
 * </p>
 * <p>
 * Using quantiles instead of min/max improves robustness with respect to
 * outliers, which occur when starting far from the desired window. Here, we ignore 25%
 * outliers on both sides.
 * </p>
 */
public class MeanLocalSolarTime extends AbstractSKControl {


    /** Sampled offset to target solar time. */
    private final List<Double> sample;
    
    /** Solar time target */
    private final double center;

    /** Duration of the ignored start part of the cycle. */
    private final double ignoredStartDuration;

    /** Search start. */
    private AbsoluteDate searchStart;

    /** Latitude at witch the local solar time will be checked*/
    private double latitude;

    private BodyShape earth;

    private boolean ascending;

    private GMODFrame gmod;

    /** Simple constructor.
     * @param name name of the control law
     * @param scalingDivisor divisor to use for scaling the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param earth Earth model
     * @param latitude latitude at which solar time should be computed
     * @param ascending if true, solar time is computed when crossing the
     * specified latitude from south to north
     * @param solarTime target solar time ((in fractional hour, i.e 9h30 = 9.5)
     * @param minSolarTime minimum accepted solar time (in fractional hour)
     * @param maxSolarTime maximum accepted solar time (in fractional hour)
     * @param checkInterval check interval, to speed up computation
     * @param ignoredStartDuration duration of the ignored start part of the cycle
     * @exception OrekitException if the UTC-TAI correction cannot be loaded
     */
    public MeanLocalSolarTime(final String name, final double scalingDivisor,
                              final String controlledName, final int controlledIndex,
                              final BodyShape earth, final double latitude, final boolean ascending,
                              final double solarTime, final double minSolarTime, final double maxSolarTime,
                              final double checkInterval, final double ignoredStartDuration)
        throws OrekitException {
        super(name, scalingDivisor, controlledName, controlledIndex, null, -1,
              0., minSolarTime, maxSolarTime);
        this.sample               = new ArrayList<Double>();
        this.center               = solarTime;
        this.ignoredStartDuration = ignoredStartDuration;
        this.latitude             = latitude;
        this.earth                = earth;
        this.ascending            = ascending;
        this.gmod                 = new GMODFrame();
    }

    /** {@inheritDoc} */
    public void initializeRun(final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, 
                              final AbsoluteDate start,
                              final AbsoluteDate end,
                              int rollingCycles) throws OrekitException {
        searchStart = start.shiftedBy(ignoredStartDuration);
        sample.clear();

        final double period = propagator.getInitialState().getKeplerianPeriod();

        // Find the first root :
        final double stepSize = period / 100;

        double dt = findFirstRoot(searchStart, stepSize, propagator);
        AbsoluteDate rootDate = searchStart.shiftedBy(dt);
        storeSample(rootDate, propagator, sample);


        // Find all other root from periodic guess :
        while (rootDate.shiftedBy(period).compareTo(end) < 0d){
            final AbsoluteDate guessDate = rootDate.shiftedBy(period);
            dt = findEncounter(guessDate, end, stepSize, period / 8d, propagator);
            if (Double.isNaN(dt)){
                // Last solution has been found :
                break;
            }
            // Re-center date for next guess
            rootDate = guessDate.shiftedBy(dt);
            // Store current point
            storeSample(rootDate, propagator, sample);
        }
    }
    
    private void storeSample(AbsoluteDate rootDate, Propagator propagator, List<Double> sample) throws OrekitException {

        if (rootDate.compareTo(searchStart) > 0) {
            // we crossed the specified latitude in the expected direction
            SpacecraftState s = propagator.propagate(rootDate);
            // compute angle between Sun and spacecraft in the equatorial plane
            final Frame gcrf =  FramesFactory.getGCRF();
            final Vector3D spacecraftPos = s.getPVCoordinates(gcrf).getPosition();
            final double time = rootDate.getComponents(TimeScalesFactory.getUTC()).getTime().getSecondsInDay();
            final double gmst     = gmod.getMeanSiderealTime(s.getDate());
            final double sunAlpha = gmst + FastMath.PI * (1 - time / (Constants.JULIAN_DAY * 0.5));
            final double dAlpha = MathUtils.normalizeAngle(spacecraftPos.getAlpha() - sunAlpha, 0);

            // convert the angle to solar time
            final double achievedSolarTime = 12.0 * (1.0 + dAlpha / FastMath.PI);

            checkLimits(achievedSolarTime);
            sample.add(achievedSolarTime);
        }     
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
        final double[] data = new double[sample.size()];
        for (int i = 0; i < data.length; ++i) {
            data[i] = sample.get(i);
        }
        final Percentile p = new Percentile();
        final double l75 = p.evaluate(data, 75.0);
        final double l25 = p.evaluate(data, 25.0);
        return FastMath.max(FastMath.abs(l75 - center), FastMath.abs(center - l25));
    }

    /** {@inheritDoc} */
    public EventDetector getEventDetector() {
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return null;
    }
    
    

    /** 
     * Find the first root to then be able to use a periodic step 
     * @param searchStart search start
     * @param stepSize step size to use
     * @param propagator propagator
     * @return the first root
     * @throws PropagationException
     */
    private double findFirstRoot(AbsoluteDate searchStart, double stepSize, Propagator propagator) throws PropagationException {
        final UnivariateFunction latitudeFunction = new LatitudeFunction(searchStart, latitude, propagator, earth);
        double previous = latitudeFunction.value(0d);
        boolean up = false;
        double root = Double.NaN;
        BrentSolver solver = new BrentSolver(1e-12);
        int i = 1;
        boolean found = false;
        while (!found){
            
            double next = latitudeFunction.value(i * stepSize);

            // Check satellite motion
            if (next - previous > 0d){
                up = true;
            }else {
                up = false;
            }
            
            // Check if bracketing has been done
            if ((previous * next <= 0d) && (up == ascending)){
                root = solver.solve(100, latitudeFunction, (i - 1) * stepSize, i * stepSize);
                // Root found
                found = true;
            }
            previous = next;
            i++;
        }
        return root;        
    }
    
    
    /**
     * Find the date at witch the latitude is crossed 
     * @param guessDate guess date from a periodic step
     * @param endDate maximal date not to overtake
     * @param shift shift value used to evaluate the latitude function bracketing around the guess date  
     * @param maxShift maximum value that the shift value can take
     * @param propagator propagator used
     * @return date at witch the satellite is crossing the latitude
     * @throws OrekitException
     * @throws NoBracketingException
     */
    private double findEncounter(final AbsoluteDate guessDate, 
                                 final AbsoluteDate endDate,
                                 final double shift,
                                 final double maxShift,
                                 final Propagator propagator)
        throws OrekitException, NoBracketingException {

        // Latitude function
        final UnivariateFunction latitudeFunction = new LatitudeFunction(guessDate, latitude, propagator, earth);
        
        // try to bracket the encounter
        double span;
        if (guessDate.shiftedBy(shift).compareTo(endDate) > 0d){
            // Take a 1e-3 security margin
            span = endDate.durationFrom(guessDate) - 1e-3;
        }else {
            span = shift;
        }
        
        while (!UnivariateRealSolverUtils.isBracketing(latitudeFunction, - span, span)) {

            if (2 * span > maxShift) {
                // let the Apache Commons Math exception be thrown
                UnivariateRealSolverUtils.verifyBracketing(latitudeFunction, - span, span);
            }else if (guessDate.shiftedBy(2 * span).compareTo(endDate) >= 1){
                // Out of range :
                return Double.NaN;
            }

            // expand the search interval
            span *= 2;

        }

        // find the encounter in the bracketed interval
        final BracketingNthOrderBrentSolver solver = new BracketingNthOrderBrentSolver(0.1, 5);
        return solver.solve(1000, latitudeFunction, - span, span);

    }
    
    
    /**
     * Function used to detect satellite latitude crossing
     */
    protected class LatitudeFunction implements UnivariateFunction{
        
        /** Propagator */
        private Propagator propagator;
        
        /** Latitude */
        private double latitude;
        
        /** Reference date used for computation*/
        private AbsoluteDate referenceDate;
        
        /** Body shape */
        private BodyShape earth;

        /**
         * Latitude function
         * @param referenceDate date of reference
         * @param latitude latitude wanted
         * @param propagator propagator used 
         * @param earth body shape
         */
        private LatitudeFunction(final AbsoluteDate referenceDate,
                                 final double latitude,
                                 final Propagator propagator,
                                 final BodyShape earth){
            this.latitude = latitude;
            this.propagator = propagator;
            this.referenceDate = referenceDate;
            this.earth = earth;
        }        


        /**
         * {@inheritDoc}
         */
        public double value(double x) {
            SpacecraftState state;
            double res = 0d;
            try {
                state = propagator.propagate(referenceDate.shiftedBy(x));
                Vector3D position = state.getPVCoordinates(earth.getBodyFrame()).getPosition();
                GeodeticPoint point = earth.transform(position, earth.getBodyFrame(), state.getDate());
                res = point.getLatitude() - latitude;
            } catch (PropagationException e) {
                e.printStackTrace();
            } catch (OrekitException e) {
                e.printStackTrace();
            }
            return res;
        }
    }
}
