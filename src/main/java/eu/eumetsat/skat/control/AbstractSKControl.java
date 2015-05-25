/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.AdapterPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.ChronologicalComparator;
import org.orekit.time.DateTimeComponents;
import org.orekit.time.TimeScalesFactory;
import org.orekit.time.TimeStamped;

import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;

/**
 * Base implementation for station-keeping control laws.
 */
public abstract class AbstractSKControl implements SKControl {

    /** Name of the control law. */
    private final String name;

    /** Maneuver models. */
    protected final TunableManeuver[] models;

    /** Sequence of dates and maneuver indices for maneuver switching */
    private final int[][] yawFlipSequence;

    /** Name of the controlled spacecraft. */
    private final String controlledName;

    /** Index of the controlled spacecraft. */
    private final int controlledIndex;

    /** Name of the reference spacecraft (may be null). */
    private final String referenceName;

    /** Index of the reference spacecraft (may be null). */
    private final int referenceIndex;

    /** Minimal value for the constraint. */
    private final double min;

    /** Maximal value for the constraint. */
    private final double max;

    /** Time horizon duration. */
    private final double horizon;

    /** Indicator of constraint violation during the cycle. */
    private double margins;

    /** Log of control law values throughout current cycle. */
    private SortedSet<TimeStamped> values;

    /** History of fitting quadratics. */
    private final List<double[]> history;

    /** Loop detection indicator. */
    private int repetition;

    /** Monitoring indicator. */
    private boolean monitoring;

    /** Simple constructor.
     * @param name name of the control law
     * @param model in-plane maneuver model
     * @param yawFlipSequence 
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param referenceName name of the reference spacecraft
     * @param referenceIndex index of the reference spacecraft
     * @param min minimal value for the constraint
     * @param max maximal value for the constraint
     * @param horizon time horizon duration
     */
    protected AbstractSKControl(final String name, final TunableManeuver[] model,
                                int[][] yawFlipSequence, final String controlledName, final int controlledIndex,
                                final String referenceName, final int referenceIndex,
                                final double min, final double max, final double horizon) {
        this.name             = name;
        this.models           = model;
        this.yawFlipSequence  = yawFlipSequence;
        this.controlledName   = controlledName;
        this.controlledIndex  = controlledIndex;
        this.referenceName    = referenceName;
        this.referenceIndex   = referenceIndex;
        this.min              = min;
        this.max              = max;
        this.horizon          = horizon;
        this.values           = new TreeSet<TimeStamped>(new ChronologicalComparator());
        this.history          = new ArrayList<double[]>();
        this.monitoring       = false;
    }

    /** {@inheritDoc} */
    public String getName() {
        return name;
    }

    /** {@inheritDoc} */
    public TunableManeuver getModel() {
    	return models[0];
    }

    /** {@inheritDoc} */
    public TunableManeuver[] getModels() {
    	return models;
    }

    /** {@inheritDoc} */
    public boolean getYawFlip(AbsoluteDate date)
    throws OrekitException {
    	
        // Requested day of year 
        final int doy = date.getComponents(TimeScalesFactory.getUTC()).getDate().getDayOfYear();
        
        // Empty model sequence: return first model
        if (yawFlipSequence.length < 2 || models.length < 2){
        	return false;
        }
        // Find the model to be applied according to the date (DoY)
        else{
        	int i;
        	for(i=0; i<yawFlipSequence.length; i++){
        		if(doy<yawFlipSequence[i][0]){
        			break;
        		}
        	}
        	// When the date is before the first model change of the sequence, apply the last one
        	if(i==0){
        		i = yawFlipSequence.length;
        	}
        	
        	return yawFlipSequence[i-1][1] == 1;
        }
    }

/** {@inheritDoc} */
    public String getControlledSpacecraftName() {
        return controlledName;
    }

    /** {@inheritDoc} */
    public int getControlledSpacecraftIndex() {
        return controlledIndex;
    }

    /** {@inheritDoc} */
    public String getReferenceSpacecraftName() {
        return referenceName;
    }

    /** {@inheritDoc} */
    public int getReferenceSpacecraftIndex() {
        return referenceIndex;
    }

    /** Reset the margins checks.
     */
    protected void resetMarginsChecks() {
        margins = Double.POSITIVE_INFINITY;
        values.clear();
    }

    /** {@inheritDoc} */
    public boolean isConstrained() {
        return (min != Double.NEGATIVE_INFINITY) || (max != Double.POSITIVE_INFINITY);
    }

    /** {@inheritDoc} */
    public double getMin() {
        return min;
    }

    /** {@inheritDoc} */
    public double getMax() {
        return max;
    }

    /** {@inheritDoc} */
    public double getMargins() {
        return margins;
    }

    /** {@inheritDoc} */
    public void setMonitoring(boolean monitoring) {
        this.monitoring = monitoring;
    }

    /** Check if we are in the monitoring phase.
     * @return true if we are in the monitoring phase
     */
    protected boolean isMonitoring() {
        return monitoring;
    }

    /** {@inheritDoc} */
    public double getMonitoredValue(final AbsoluteDate date) {
        if (values.isEmpty()) {
            return 0;
        } else if (date.compareTo(values.first().getDate()) <= 0) {
            return ((DateValue) values.first()).getValue();
        } else if (date.compareTo(values.last().getDate()) >= 0) {
            return ((DateValue) values.last()).getValue();
        } else {
            return ((DateValue) values.tailSet(date).first()).getValue();
        }
    }

    /** {@inheritDoc} */
    public double getTimeHorizon() {
        return horizon;
    }

    /** Check if control limits are exceeded and add value to monitored list.
     * @param date current date
     * @param value current value of the control law
     */
    protected void checkMargins(final AbsoluteDate date, final double value) {
        values.add(new DateValue(date, value));
        if (isConstrained()) {
            margins = FastMath.min(margins, FastMath.min(value - min, max - value));
        } else {
            margins = 0.0;
        }
    }

    /** Find the longest maneuver-free interval.
     * <p>
     * Only maneuvers matching the control law model maneuver are considered here.
     * </p>
     * @param maneuvers maneuvers scheduled for this control law
     * @param fixedManeuvers list of maneuvers already fixed for the cycle
     * @param start start date of the propagation
     * @param end end date of the propagation
     * @return longest maneuver-free interval
     * @throws OrekitException 
     */
    protected AbsoluteDate[] getManeuverFreeInterval(final ScheduledManeuver[] maneuvers,
                                                     final List<ScheduledManeuver> fixedManeuvers,
                                                     final AbsoluteDate start, final AbsoluteDate end) throws OrekitException {

        // gather all special dates (start, end, maneuvers) in one chronologically sorted set
        SortedSet<AbsoluteDate> sortedDates = new TreeSet<AbsoluteDate>();
        sortedDates.add(start);
        sortedDates.add(end);
        AbsoluteDate last = start;
        for (final ScheduledManeuver maneuver : maneuvers) {
            final AbsoluteDate date = maneuver.getDate();
            final TunableManeuver model = getModel();
            if (maneuver.getName().equals(model.getName()) && date.compareTo(last) >= 0) {
                // this is the last maneuver seen so far
                last = date;
            }
            sortedDates.add(date);
        }
        for (final ScheduledManeuver maneuver : fixedManeuvers) {
            final AbsoluteDate date = maneuver.getDate();
            final TunableManeuver model = getModel();
            if (maneuver.getName().equals(model.getName()) && date.compareTo(last) >= 0) {
                // this is the last maneuver seen so far
                last = date;
            }
            sortedDates.add(date);
        }

        // select the longest maneuver-free interval after last in plane maneuver
        AbsoluteDate freeIntervalStart = last;
        AbsoluteDate freeIntervalEnd   = last;
        AbsoluteDate previousDate      = last;
        for (final AbsoluteDate currentDate : sortedDates) {
            if (previousDate.compareTo(last) >= 0 &&
                currentDate.durationFrom(previousDate) > freeIntervalEnd.durationFrom(freeIntervalStart)) {
                freeIntervalStart = previousDate;
                freeIntervalEnd   = currentDate;
            }
            previousDate = currentDate;
        }

        // prevent propagating after end date
        if (freeIntervalStart.compareTo(end) >= 0) {
            freeIntervalStart = end;
        }
        if (freeIntervalEnd.compareTo(end) >= 0) {
            freeIntervalEnd = end;
        }

        return new AbsoluteDate[] {
            freeIntervalStart, freeIntervalEnd
        };

    }

    /** Get the sign of the maneuver model with respect to orbital momentum.
     * @param state spacecraft state
     * @return +1 if model thrust is along momentum direction, -1 otherwise
     * @throws OrekitException 
     */
    protected double thrustSignMomentum(final SpacecraftState state, final TunableManeuver model) throws OrekitException {
        final Vector3D thrustDirection =
                state.getAttitude().getRotation().applyInverseTo(model.getDirection());
        Vector3D momentum = state.getPVCoordinates().getMomentum();
        return FastMath.signum(Vector3D.dotProduct(thrustDirection, momentum));
    }

    /** Get the sign of the maneuver model with respect to orbital velocity.
     * @param state spacecraft state
     * @return +1 if model thrust is along velocity direction, -1 otherwise
     * @throws OrekitException 
     */
    protected double thrustSignVelocity(final SpacecraftState state, final TunableManeuver model) throws OrekitException {
        final Vector3D thrustDirection =
                state.getAttitude().getRotation().applyInverseTo(model.getDirection());
        Vector3D velocity = state.getPVCoordinates().getVelocity();
        return FastMath.signum(Vector3D.dotProduct(thrustDirection, velocity));
    }

    /** Change the trajectory of some maneuvers.
     * @param maneuvers maneuvers array
     * @param from index of the first maneuver to update (included)
     * @param to index of the last maneuver to update (excluded)
     * @param trajectory trajectory to use for maneuvers
     */
    protected void changeTrajectory(final ScheduledManeuver[] maneuvers,
                                    final int from, final int to,
                                    final AdapterPropagator adapterPropagator) {
        for (int i = from; i < to; ++i) {
            maneuvers[i] = new ScheduledManeuver(maneuvers[i].getModel(), maneuvers[i].getDate(),
                                                 maneuvers[i].getDeltaV(), maneuvers[i].getThrust(),
                                                 maneuvers[i].getIsp(), adapterPropagator,
                                                 maneuvers[i].isReplanned());
        }
    }

    /** Distribute a velocity increment change over non-saturated maneuvers.
     * @param dV velocity increment change to distribute
     * @param dT date change to apply to all maneuvers
     * @param maneuvers array of maneuvers to change
     * @param adapterPropagator propagator to use for maneuvers
     * @throws OrekitException 
     */
    protected void distributeDV(final double dV, final double dT,
                                final ScheduledManeuver[] maneuvers,
                                final AdapterPropagator adapterPropagator) throws OrekitException {


        double remaining = dV;
        while (FastMath.abs(remaining) > 1.e-6) {

            // identify the maneuvers that can be changed
            final List<Integer> nonSaturated = new ArrayList<Integer>(maneuvers.length);
            final TunableManeuver[] models = getModels();
            for (int i = 0; i < maneuvers.length; ++i) {
            	for(final TunableManeuver model : models){
	                if (maneuvers[i].getName().equals(model.getName()) && !maneuvers[i].isSaturated(dV)) {
	                    nonSaturated.add(i);
	                }
                }
            }

            if (nonSaturated.isEmpty()) {
                // we cannot do anything more
                return;
            }

            // distribute the remaining dV evenly
            final double dVPart = remaining / nonSaturated.size();
            for (final int i : nonSaturated) {
                final TunableManeuver model = maneuvers[i].getModel();
                final double original = maneuvers[i].getSignedDeltaV();
                final double changed  = FastMath.max(model.getCurrentDVInf(), FastMath.min(model.getCurrentDVSup(), original + dVPart));
                remaining   -= changed - original;
                maneuvers[i] = new ScheduledManeuver(maneuvers[i].getModel(),
                                                     maneuvers[i].getDate().shiftedBy(dT),
                                                     new Vector3D(changed, model.getDirection()),
                                                     maneuvers[i].getThrust(),
                                                     maneuvers[i].getIsp(),
                                                     adapterPropagator,
                                                     maneuvers[i].isReplanned());
            }

        }

    }

    /** Clear the history of quadratic fittings.
     */
    protected void clearHistory() {
        history.clear();
        repetition = 0;
    }

    /** Add a quadratic fit to the history
     * @param quadratic parameters of a quadratic fit to add
     * @param start start of the fit interval
     * @param end end of the fit interval
     */
    protected void addQuadraticFit(final double[] quadratic, final double start, final double end) {

        // check for loops
        boolean loopDetected = false;
        final double referenceValue = getMax(quadratic[0], quadratic[1], quadratic[2], start, end);
        for (final double[] old : history) {
            final double maxDelta = getMax(old[0] - quadratic[0], old[1] - quadratic[1], old[2] - quadratic[2],
                                           start, end);
            if (maxDelta < 1.0e-6 * referenceValue) {
                // we have found again a quadratic we have already seen
                loopDetected = true;
            }
        }

        if (loopDetected) {
            ++repetition;
        }

        history.add(quadratic.clone());

    }

    /** Get the maximal absolute value of a quadratic over an interval.
     * @param a0 constant term of the quadratic
     * @param a1 slope term of the quadratic
     * @param a2 acceleration term of the quadratic
     * @param start start of the interval
     * @param end end of the interval
     * @return maximal absolute value of the quadratic
     */
    private double getMax(final double a0, final double a1, final double a2, final double start, final double end) {

        // absolute value at start
        double max = FastMath.abs(a0 + a1 * start + a2 * start * start);

        // absolute value at end
        max = FastMath.max(max, FastMath.abs(a0 + a1 * end + a2 * end * end));

        final double tPeak = -a1 / (2 * a2);
        if (tPeak > start && tPeak < end) {
            // absolute value at peak
            max = FastMath.max(max, FastMath.abs(a0 + a1 * tPeak + a2 * tPeak * tPeak));
        }

        return max;

    }

    /** Check if a loop has been found in the successive quadratic fits.
     * @return true if a loop has been found
     */
    protected boolean loopDetected() {
        return repetition > 10;
    }

    /** Inner container for date/value pairs. */
    private static class DateValue implements TimeStamped, Comparable<DateValue> {

        /** Date. */
        private final AbsoluteDate date;

        /** Value. */
        private final double value;

        /** Simple constructor.
         * @param date current date
         * @param value current value
         */
        public DateValue(final AbsoluteDate date, final double value) {
            this.date  = date;
            this.value = value;
        }

        /** {@inheritDoc} */
        public AbsoluteDate getDate() {
            return date;
        }

        /** Get the current value.
         * @return current value
         */
        public double getValue() {
            return value;
        }

        /** {@inheritDoc} */
        public int compareTo(final DateValue other) {
            return date.compareTo(other.date);
        }

        /** {@inheritDoc} */
        public boolean equals(final Object other) {

            if (other == this) {
                // first fast check
                return true;
            }

            if ((other != null) && (other instanceof DateValue)) {
                return date.equals(((DateValue) other).date);
            }

            return false;

        }

    }
}
