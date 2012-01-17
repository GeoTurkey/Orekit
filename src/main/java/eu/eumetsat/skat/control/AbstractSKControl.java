/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

import org.apache.commons.math.analysis.UnivariateFunction;
import org.apache.commons.math.analysis.solvers.BaseUnivariateRealSolver;
import org.apache.commons.math.analysis.solvers.BracketingNthOrderBrentSolver;
import org.apache.commons.math.exception.NoBracketingException;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.AdapterPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinates;

import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.OrekitWrapperException;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;



/**
 * Base implementation for station-keeping control laws.
 */
public abstract class AbstractSKControl implements SKControl {

    /** Name of the control law. */
    private final String name;

    /** Maneuver model. */
    private final TunableManeuver model;

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

    /** Simple constructor.
     * @param name name of the control law
     * @param model in-plane maneuver model
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param referenceName name of the reference spacecraft
     * @param referenceIndex index of the reference spacecraft
     * @param min minimal value for the constraint
     * @param max maximal value for the constraint
     * @param horizon time horizon duration
     */
    protected AbstractSKControl(final String name, final TunableManeuver model,
                                final String controlledName, final int controlledIndex,
                                final String referenceName, final int referenceIndex,
                                final double min, final double max, final double horizon) {
        this.name            = name;
        this.model           = model;
        this.controlledName  = controlledName;
        this.controlledIndex = controlledIndex;
        this.referenceName   = referenceName;
        this.referenceIndex  = referenceIndex;
        this.min             = min;
        this.max             = max;
        this.horizon         = horizon;
    }

    /** {@inheritDoc} */
    public String getName() {
        return name;
    }

    /** Get the maneuver model.
     * @return maneuver model
     */
    public TunableManeuver getModel() {
        return model;
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
    public double getTimeHorizon() {
        return horizon;
    }

    /** Check if control limits are exceeded.
     * @param value current value of the control law
     */
    protected void checkMargins(final double value) {
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
     */
    protected AbsoluteDate[] getManeuverFreeInterval(final ScheduledManeuver[] maneuvers,
                                                     final List<ScheduledManeuver> fixedManeuvers,
                                                     final AbsoluteDate start, final AbsoluteDate end) {

        // gather all special dates (start, end, maneuvers) in one chronologically sorted set
        SortedSet<AbsoluteDate> sortedDates = new TreeSet<AbsoluteDate>();
        sortedDates.add(start);
        sortedDates.add(end);
        AbsoluteDate last = start;
        for (final ScheduledManeuver maneuver : maneuvers) {
            final AbsoluteDate date = maneuver.getDate();
            if (maneuver.getName().equals(model.getName()) && date.compareTo(last) >= 0) {
                // this is the last maneuver seen so far
                last = date;
            }
            sortedDates.add(date);
        }
        for (final ScheduledManeuver maneuver : fixedManeuvers) {
            final AbsoluteDate date = maneuver.getDate();
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
     */
    protected double thrustSignMomentum(final SpacecraftState state) {
        final Vector3D thrustDirection =
                state.getAttitude().getRotation().applyInverseTo(getModel().getDirection());
        Vector3D momentum = state.getPVCoordinates().getMomentum();
        return FastMath.signum(Vector3D.dotProduct(thrustDirection, momentum));
    }

    /** Get the sign of the maneuver model with respect to orbital velocity.
     * @param state spacecraft state
     * @return +1 if model thrust is along velocity direction, -1 otherwise
     */
    protected double thrustSignVelocity(final SpacecraftState state) {
        final Vector3D thrustDirection =
                state.getAttitude().getRotation().applyInverseTo(getModel().getDirection());
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
     */
    protected void distributeDV(final double dV, final double dT,
                                final ScheduledManeuver[] maneuvers,
                                final AdapterPropagator adapterPropagator) {

        final double inf = model.getDVInf();
        final double sup = model.getDVSup();

        double remaining = dV;
        while (FastMath.abs(remaining) > model.getEliminationThreshold()) {

            // identify the maneuvers that can be changed
            final List<Integer> nonSaturated = new ArrayList<Integer>(maneuvers.length);
            for (int i = 0; i < maneuvers.length; ++i) {
                if (maneuvers[i].getName().equals(model.getName()) &&
                    ((dV < 0 && maneuvers[i].getSignedDeltaV() > inf) ||
                     (dV > 0 && maneuvers[i].getSignedDeltaV() < sup))) {
                    nonSaturated.add(i);
                }
            }

            if (nonSaturated.isEmpty()) {
                // we cannot do anything more
                return;
            }

            // distribute the remaining dV evenly
            final double dVPart = remaining / nonSaturated.size();
            for (final int i : nonSaturated) {
                final double original = maneuvers[i].getSignedDeltaV();
                final double changed  = FastMath.max(inf, FastMath.min(sup, original + dVPart));
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

    /** 
     * Find the first crossing of the reference latitude.
     * @param latitude latitude to search for
     * @param ascending indicator for desired crossing direction
     * @param earth Earth model
     * @param searchStart search start
     * @param end maximal date not to overtake
     * @param stepSize step size to use
     * @param propagator propagator
     * @return first crossing
     * @exception OrekitException if state cannot be propagated
     * @exception SkatException if latitude is never crossed
     * @see #latitudeCrossing(double, BodyShape, AbsoluteDate, AbsoluteDate, double, double, Propagator)
     */
    protected SpacecraftState firstLatitudeCrossing(final double latitude, final boolean ascending,
                                                    final BodyShape earth,
                                                    final AbsoluteDate searchStart, final AbsoluteDate end,
                                                    final double stepSize, final Propagator propagator)
        throws OrekitException, SkatException {

        double previousLatitude = Double.NaN;
        for (AbsoluteDate date = searchStart; date.compareTo(end) < 0; date = date.shiftedBy(stepSize)) {
            final PVCoordinates pv       = propagator.propagate(date).getPVCoordinates(earth.getBodyFrame());
            final double currentLatitude = earth.transform(pv.getPosition(), earth.getBodyFrame(), date).getLatitude();
            if (((previousLatitude <= latitude) && (currentLatitude >= latitude) &&  ascending) ||
                ((previousLatitude >= latitude) && (currentLatitude <= latitude) && !ascending)) {
                return latitudeCrossing(latitude, earth, date.shiftedBy(-0.5 * stepSize), end,
                                        0.5 * stepSize, 2 * stepSize, propagator);
            }
            previousLatitude = currentLatitude;
        }

        throw new SkatException(SkatMessages.LATITUDE_NEVER_CROSSED,
                                FastMath.toDegrees(latitude), searchStart, end);

    }

    /**
     * Find the state at which a reference latitude is crossed.
     * @param latitude latitude to search for
     * @param earth Earth model
     * @param guessDate guess date for the crossing
     * @param endDate maximal date not to overtake
     * @param shift shift value used to evaluate the latitude function bracketing around the guess date  
     * @param maxShift maximum value that the shift value can take
     * @param propagator propagator used
     * @return state at latitude crossing time
     * @throws OrekitException if state cannot be propagated
     * @throws NoBracketingException if latitude cannot be bracketed in the search interval
     * @see #firstLatitudeCrossing(double, boolean, BodyShape, AbsoluteDate, AbsoluteDate, double, Propagator)
     */
    protected SpacecraftState latitudeCrossing(final double latitude, final BodyShape earth,
                                               final AbsoluteDate guessDate, final AbsoluteDate endDate,
                                               final double shift, final double maxShift,
                                               final Propagator propagator)
        throws OrekitException, NoBracketingException {

        // function evaluating to 0 at latitude crossings
        final UnivariateFunction latitudeFunction = new UnivariateFunction() {
            /** {@inheritDoc} */
            public double value(double x) throws OrekitWrapperException {
                try {
                    final SpacecraftState state = propagator.propagate(guessDate.shiftedBy(x));
                    final Vector3D position = state.getPVCoordinates(earth.getBodyFrame()).getPosition();
                    final GeodeticPoint point = earth.transform(position, earth.getBodyFrame(), state.getDate());
                    return point.getLatitude() - latitude;
                } catch (OrekitException oe) {
                    throw new OrekitWrapperException(oe);
                }
            }
        };

        // find the latitude crossing
        return findZero(latitudeFunction, guessDate, endDate, shift, maxShift, propagator);

    }

    /**
     * Find the state at which a reference longitude is crossed.
     * @param longitude longitude to search for
     * @param earth Earth model
     * @param guessDate guess date for the crossing
     * @param endDate maximal date not to overtake
     * @param shift shift value used to evaluate the longitude function bracketing around the guess date  
     * @param maxShift maximum value that the shift value can take
     * @param propagator propagator used
     * @return state at longitude crossing time
     * @throws OrekitException if state cannot be propagated
     * @throws NoBracketingException if longitude cannot be bracketed in the search interval
     * @see #latitudeCrossing(double, boolean, BodyShape, AbsoluteDate, AbsoluteDate, double, Propagator)
     */
    protected SpacecraftState longitudeCrossing(final double longitude, final BodyShape earth,
                                                final AbsoluteDate guessDate, final AbsoluteDate endDate,
                                                final double shift, final double maxShift,
                                                final Propagator propagator)
        throws OrekitException, NoBracketingException {

        // function evaluating to 0 at longitude crossings
        final UnivariateFunction longitudeFunction = new UnivariateFunction() {
            /** {@inheritDoc} */
            public double value(double x) throws OrekitWrapperException {
                try {
                    final SpacecraftState state = propagator.propagate(guessDate.shiftedBy(x));
                    final Vector3D position = state.getPVCoordinates(earth.getBodyFrame()).getPosition();
                    final GeodeticPoint point = earth.transform(position, earth.getBodyFrame(), state.getDate());
                    return MathUtils.normalizeAngle(point.getLongitude() - longitude, 0);
                } catch (OrekitException oe) {
                    throw new OrekitWrapperException(oe);
                }
            }
        };

        // find the longitude crossing
        return findZero(longitudeFunction, guessDate, endDate, shift, maxShift, propagator);

    }

    /**
     * Find the state at which a function becomes zero.
     * @param function function to check
     * @param guessDate guess date for the crossing
     * @param endDate maximal date not to overtake
     * @param shift shift value used to evaluate the function bracketing around the guess date  
     * @param maxShift maximum value that the shift value can take
     * @param propagator propagator used
     * @return state at function zero time
     * @throws OrekitException if state cannot be propagated
     * @throws NoBracketingException if function cannot be bracketed in the search interval
     */
    private SpacecraftState findZero(final UnivariateFunction function,
                                     final AbsoluteDate guessDate, final AbsoluteDate endDate,
                                     final double shift, final double maxShift,
                                     final Propagator propagator)
        throws OrekitException, NoBracketingException {

        try {

            // try to bracket the encounter
            double span;
            if (guessDate.shiftedBy(shift).compareTo(endDate) > 0) {
                // Take a 1e-3 security margin
                span = endDate.durationFrom(guessDate) - 1e-3;
            } else {
                span = shift;
            }

            double earliestPositive = Double.POSITIVE_INFINITY;
            double latestPositive   = Double.NEGATIVE_INFINITY;
            double earliestNegative = Double.POSITIVE_INFINITY;
            double latestNegative   = Double.NEGATIVE_INFINITY;
            do {
                final double mValue = function.value(-span);
                final double pValue = function.value(+span);
                if (mValue <= 0) {
                    earliestNegative = FastMath.min(earliestNegative, -span);
                    latestNegative   = FastMath.max(latestNegative,   -span);
                } else {
                    earliestPositive = FastMath.min(earliestPositive, -span);
                    latestPositive   = FastMath.max(latestPositive,   -span);

                }
                if (pValue <= 0) {
                    earliestNegative = FastMath.min(earliestNegative, +span);
                    latestNegative   = FastMath.max(latestNegative,   +span);
                } else {
                    earliestPositive = FastMath.min(earliestPositive, +span);
                    latestPositive   = FastMath.max(latestPositive,   +span);

                }

                if ((Double.isInfinite(earliestNegative) && Double.isInfinite(latestNegative)) ||
                    (Double.isInfinite(earliestPositive) && Double.isInfinite(latestPositive))) {

                    if (guessDate.shiftedBy(2 * span).compareTo(endDate) > 0) {
                        // out of range
                        return null;
                    } else {
                        // expand the search interval
                        span *= 2;
                    }

                } else {

                    // find the latitude crossing in the bracketed interval
                    final BaseUnivariateRealSolver<UnivariateFunction> solver =
                            new BracketingNthOrderBrentSolver(0.1, 5);
                    final double dEnEp = FastMath.abs(earliestPositive - earliestNegative);
                    final double dEnLp = FastMath.abs(latestPositive   - earliestNegative);
                    final double dLnEp = FastMath.abs(earliestPositive - latestNegative);
                    final double dLnLp = FastMath.abs(latestPositive   - latestNegative);
                    final double selectedNegative;
                    final double selectedPositive;
                    if (dEnEp < dEnLp) {
                        if (dEnEp < dLnEp) {
                            if (dEnEp < dLnLp) {
                                selectedNegative = earliestNegative;
                                selectedPositive = earliestPositive;
                            } else {
                                selectedNegative = latestNegative;
                                selectedPositive = latestPositive;
                            }
                        } else {
                            if (dLnEp < dLnLp) {
                                selectedNegative = latestNegative;
                                selectedPositive = earliestPositive;
                            } else {
                                selectedNegative = latestNegative;
                                selectedPositive = latestPositive;
                            }
                        }
                    } else {
                        if (dEnLp < dLnEp) {
                            if (dEnLp < dLnLp) {
                                selectedNegative = earliestNegative;
                                selectedPositive = latestPositive;
                            } else {
                                selectedNegative = latestNegative;
                                selectedPositive = latestPositive;
                            }
                        } else {
                            if (dLnEp < dLnLp) {
                                selectedNegative = latestNegative;
                                selectedPositive = earliestPositive;
                            } else {
                                selectedNegative = latestNegative;
                                selectedPositive = latestPositive;
                            }
                        }
                    }
                    final double dt = (selectedNegative <= selectedPositive) ?
                            solver.solve(1000, function, selectedNegative, selectedPositive) :
                            solver.solve(1000, function, selectedPositive, selectedNegative);

                    return propagator.propagate(guessDate.shiftedBy(dt));

                }

            } while (2 * span < maxShift);

            // no bracketing found
            throw new NoBracketingException(-span, span, function.value(-span), function.value(+span));

        } catch (OrekitWrapperException owe) {
            throw owe.getWrappedException();
        }

    }
    
}
