/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.analysis.UnivariateFunction;
import org.apache.commons.math.analysis.solvers.BaseUnivariateRealSolver;
import org.apache.commons.math.analysis.solvers.BracketingNthOrderBrentSolver;
import org.apache.commons.math.analysis.solvers.UnivariateRealSolverUtils;
import org.apache.commons.math.exception.NoBracketingException;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.ManeuverAdapterPropagator;
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

    /** Name of the controlled spacecraft. */
    private final String controlledName;

    /** Index of the controlled spacecraft. */
    private final int controlledIndex;

    /** Name of the reference spacecraft (may be null). */
    private final String referenceName;

    /** Index of the reference spacecraft (may be null). */
    private final int referenceIndex;

    /** Minimal value for the residual. */
    private final double min;

    /** Maximal value for the residual. */
    private final double max;

    /** Indicator of constraint violation during the cycle. */
    private double margins;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param referenceName name of the reference spacecraft
     * @param referenceIndex index of the reference spacecraft
     * @param min minimal value for the residual
     * @param max maximal value for the residual
     */
    protected AbstractSKControl(final String name,
                                final String controlledName, final int controlledIndex,
                                final String referenceName, final int referenceIndex,
                                final double min, final double max) {
        this.name            = name;
        this.controlledName  = controlledName;
        this.controlledIndex = controlledIndex;
        this.referenceName   = referenceName;
        this.referenceIndex  = referenceIndex;
        this.min             = min;
        this.max             = max;
    }

    /** {@inheritDoc} */
    public String getName() {
        return name;
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

    /** Change the trajectory of some maneuvers.
     * @param maneuvers maneuvers array
     * @param from index of the first maneuver to update (included)
     * @param to index of the last maneuver to update (excluded)
     * @param trajectory trajectory to use for maneuvers
     */
    protected void changeTrajectory(final ScheduledManeuver[] maneuvers,
                                    final int from, final int to,
                                    final ManeuverAdapterPropagator adapterPropagator) {
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
     * @param model model of the maneuvers to change
     * @param maneuvers array of maneuvers to change
     * @param adapterPropagator propagator to use for maneuvers
     */
    protected void distributeDV(final double dV, final double dT,
                                final TunableManeuver model, final ScheduledManeuver[] maneuvers,
                                final ManeuverAdapterPropagator adapterPropagator) {

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
     * @throws OrekitException if state cannot be propagated
     */
    protected SpacecraftState findFirstCrossing(final double latitude, final boolean ascending,
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
                return findLatitudeCrossing(latitude, earth, date.shiftedBy(-0.5 * stepSize), end,
                                            0.5 * stepSize, 2 * stepSize, propagator);
            }
            previousLatitude = currentLatitude;
        }

        throw new SkatException(SkatMessages.LATITUDE_NEVER_CROSSED,
                                FastMath.toDegrees(latitude), searchStart, end);

    }
    
    
    /**
     * Find the state at which the reference latitude is crossed.
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
     */
    protected SpacecraftState findLatitudeCrossing(final double latitude, final BodyShape earth,
                                                   final AbsoluteDate guessDate, final AbsoluteDate endDate,
                                                   final double shift, final double maxShift,
                                                   final Propagator propagator)
        throws OrekitException, NoBracketingException {

        try {

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

            // try to bracket the encounter
            double span;
            if (guessDate.shiftedBy(shift).compareTo(endDate) > 0) {
                // Take a 1e-3 security margin
                span = endDate.durationFrom(guessDate) - 1e-3;
            } else {
                span = shift;
            }

            while (!UnivariateRealSolverUtils.isBracketing(latitudeFunction, -span, span)) {

                if (2 * span > maxShift) {
                    // let the Apache Commons Math exception be thrown
                    UnivariateRealSolverUtils.verifyBracketing(latitudeFunction, -span, span);
                } else if (guessDate.shiftedBy(2 * span).compareTo(endDate) > 0) {
                    // Out of range :
                    return null;
                }

                // expand the search interval
                span *= 2;

            }

            // find the encounter in the bracketed interval
            final BaseUnivariateRealSolver<UnivariateFunction> solver =
                    new BracketingNthOrderBrentSolver(0.1, 5);
            final double dt = solver.solve(1000, latitudeFunction,-span, span);
            return propagator.propagate(guessDate.shiftedBy(dt));

        } catch (OrekitWrapperException owe) {
            throw owe.getWrappedException();
        }

    }
    
}
