/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BaseUnivariateSolver;
import org.apache.commons.math3.analysis.solvers.BracketingNthOrderBrentSolver;
import org.apache.commons.math3.exception.NoBracketingException;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.AdapterPropagator;
import org.orekit.propagation.analytical.J2DifferentialEffect;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinates;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.OrekitWrapperException;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Base implementation for station-keeping control laws.
 */
public abstract class AbstractLeoSKControl extends AbstractSKControl {

    /** Iteration number. */
    protected int iteration;

    /** Mean period. */
    protected double meanPeriod;

    /** Cycle start. */
    protected AbsoluteDate cycleStart;

    /** Cycle end. */
    protected AbsoluteDate cycleEnd;

    /** Reference radius of the Earth for the potential model. */
    protected final double referenceRadius;

    /** Central attraction coefficient. */
    protected final double mu;

    /** Un-normalized zonal coefficient. */
    protected final double j2;

    /** Maximum number of maneuvers to set up in one cycle. */
    protected final int maxManeuvers;

    /** Time offset of the first maneuver with respect to cycle start. */
    protected final double firstOffset;

    /** Minimum time between split parts in number of orbits. */
    protected final int orbitsSeparation;

    /** Earth model. */
    protected final OneAxisEllipsoid earth;

    /** Sun model. */
    private final CelestialBody sun;

    /** Maneuver-free interval. */
    protected AbsoluteDate[] freeInterval;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param model in-plane maneuver model
     * @param firstOffset time offset of the first maneuver with respect to cycle start
     * @param maxManeuvers maximum number of maneuvers to set up in one cycle
     * @param orbitsSeparation minimum time between split parts in number of orbits
     * @param earth Earth model
     * @param sun Sun model
     * @param referenceRadius reference radius of the Earth for the potential model (m)
     * @param mu central attraction coefficient (m<sup>3</sup>/s<sup>2</sup>)
     * @param j2 un-normalized zonal coefficient (about +1.08e-3 for Earth)
     * @param min minimal value for the constraint
     * @param max maximal value for the constraint
     * @param horizon time horizon duration
     */
    protected AbstractLeoSKControl(final String name, final String controlledName, final int controlledIndex,
                                   final TunableManeuver model, final double firstOffset, final int maxManeuvers,
                                   final int orbitsSeparation, final OneAxisEllipsoid earth,
                                   final CelestialBody sun,
                                   final double referenceRadius, final double mu, final double j2,
                                   final double min, final double max, final double horizon) {
        super(name, model, controlledName, controlledIndex, null, -1, min, max, horizon);
        this.firstOffset      = firstOffset;
        this.maxManeuvers     = maxManeuvers;
        this.orbitsSeparation = orbitsSeparation;
        this.earth            = earth;
        this.sun              = sun;
        this.referenceRadius  = referenceRadius;
        this.mu               = mu;
        this.j2               = j2;
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
     * @return first crossing (may be null)
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
     * @return state at latitude crossing time (may be null)
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
     * @return state at longitude crossing time (may be null)
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
     * @param guess guess for the free variable
     * @param min lower bound for free variable
     * @param max upper bound for free variable
     * @param initialSpan initial span used to bracket root around the guess date  
     * @param maxSpan maximum span to use before declaring bracketing failed
     * @param convergence convergence threshold for solver
     * @return root of the function (may be NaN if out of range is detected)
     * @exception OrekitException if the function throws one
     * @exception NoBracketingException if function cannot be bracketed in the search interval
     */
    protected double findZero(final UnivariateFunction function, final double guess,
                              final double min, final double max,
                              final double initialSpan, final double maxSpan,
                              final double convergence)
        throws OrekitException, NoBracketingException {

        try {

            // try to bracket the root
            double span = initialSpan;

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

                    if (guess - 2 * span < min || guess + 2 * span > max) {
                        // out of range
                        return Double.NaN;
                    } else {
                        // expand the search interval
                        span *= 2;
                    }

                } else {

                    // find the root in the bracketed interval
                    final BaseUnivariateSolver<UnivariateFunction> solver =
                            new BracketingNthOrderBrentSolver(convergence, 5);
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
                    return (selectedNegative <= selectedPositive) ?
                            solver.solve(1000, function, selectedNegative, selectedPositive) :
                            solver.solve(1000, function, selectedPositive, selectedNegative);

                }

            } while (2 * span < maxSpan);

            // no bracketing found
            throw new NoBracketingException(-span, span, function.value(-span), function.value(+span));

        } catch (OrekitWrapperException owe) {
            throw owe.getWrappedException();
        }

    }

    /**
     * Find the state at which a function becomes zero.
     * @param function function to check
     * @param guessDate guess date for the crossing
     * @param endDate maximal date not to overtake
     * @param shift shift value used to evaluate the function bracketing around the guess date  
     * @param maxShift maximum value that the shift value can take
     * @param propagator propagator used
     * @return state at function zero time (may be null)
     * @throws OrekitException if state cannot be propagated
     * @throws NoBracketingException if function cannot be bracketed in the search interval
     */
    private SpacecraftState findZero(final UnivariateFunction function,
                                     final AbsoluteDate guessDate, final AbsoluteDate endDate,
                                     final double shift, final double maxShift,
                                     final Propagator propagator)
        throws OrekitException, NoBracketingException {

        final double max = endDate.durationFrom(guessDate) - 1.0e-3;
        final double dt = findZero(function, 0, Double.NEGATIVE_INFINITY, max,
                                   FastMath.min(shift, max), maxShift, 0.1);
        return Double.isNaN(dt) ? null : propagator.propagate(guessDate.shiftedBy(dt));
    }

    protected ScheduledManeuver[] tuneInclinationManeuver(final ScheduledManeuver[] tunables,
                                                          final BoundedPropagator reference,
                                                          final SpacecraftState nodeState,
                                                          final double deltaI,
                                                          final boolean compensateLongBurn)
        throws OrekitException {

        ScheduledManeuver[] tuned = new ScheduledManeuver[0];
        final AdapterPropagator adapterPropagator = new AdapterPropagator(reference);
        if (iteration == 0) {
            // we need to first define the number of maneuvers and their initial settings

            // compute the out of plane maneuver required to get the initial inclination offset
            final Vector3D v = nodeState.getPVCoordinates().getVelocity();
            final double totalDeltaV = thrustSignMomentum(nodeState) * FastMath.signum(v.getZ()) * v.getNorm() * deltaI;
            
            // compute the number of maneuvers required
            final double separation = orbitsSeparation * meanPeriod;
            final TunableManeuver model = getModel();
            final double limitDV = (totalDeltaV < 0) ? model.getDVInf() : model.getDVSup();
            int nMan = FastMath.min(maxManeuvers, (int) FastMath.ceil(FastMath.abs(totalDeltaV / limitDV)));
            while (nodeState.getDate().shiftedBy((nMan - 1) * separation).getDate().compareTo(cycleEnd) >= 0) {
                --nMan;
            }
            final double deltaV = FastMath.max(model.getDVInf(), FastMath.min(model.getDVSup(), totalDeltaV / nMan));

            tuned = new ScheduledManeuver[tunables.length + nMan];
            System.arraycopy(tunables, 0, tuned, 0, tunables.length);
            changeTrajectory(tuned, 0, tunables.length, adapterPropagator);

            // add the new maneuvers
            for (int i = 0; i < nMan; ++i) {
                tuned[tunables.length + i] = new ScheduledManeuver(model,
                                                                   nodeState.getDate().shiftedBy(i * separation),
                                                                   new Vector3D(deltaV, model.getDirection()),
                                                                   model.getCurrentThrust(), model.getCurrentISP(),
                                                                   adapterPropagator, false);
            }

        } else {

            // compute the out of plane maneuver required to get the initial inclination offset
            final Vector3D v = nodeState.getPVCoordinates().getVelocity();
            final double deltaVChange = thrustSignMomentum(nodeState) * FastMath.signum(v.getZ()) * v.getNorm() * deltaI;

            // distribute the change over all maneuvers
            tuned = tunables.clone();
            changeTrajectory(tuned, 0, tuned.length, adapterPropagator);
            distributeDV(deltaVChange, 0.0, tuned, adapterPropagator);

        }

        if (compensateLongBurn) {
            for (int i = 0; i < tuned.length; ++i) {
                if (tuned[i].getName().equals(getModel().getName())) {
                    tuned[i] = longBurnCompensation(tuned[i]);
                }
            }
        }

        // finalize propagator
        for (final ScheduledManeuver maneuver : tuned) {
            final SpacecraftState state = maneuver.getStateBefore();
            AdapterPropagator.DifferentialEffect directEffect =
                    new SmallManeuverAnalyticalModel(state, maneuver.getDeltaV(), maneuver.getIsp());
            AdapterPropagator.DifferentialEffect resultEffect =
                    new J2DifferentialEffect(state, directEffect, false, referenceRadius, mu, j2);
            adapterPropagator.addEffect(directEffect);
            adapterPropagator.addEffect(resultEffect);
        }

        return tuned;

    }

    /** Compensate inefficiency of long burns.
     *  <p>
     *  For a long out-of-plane maneuver, Isp has to be adapted to reflect the
     *  fact more mass will be consumed to achieve the same velocity increment.
     *  </p>
     *  @param maneuver maneuver to compensate
     *  @return compensated maneuver (Isp reduced to get same dV with more consumed mass)
     *  @throws PropagationException if state cannot be propagated around maneuvera
     */
    private ScheduledManeuver longBurnCompensation(final ScheduledManeuver maneuver) throws PropagationException {
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
        while( alphaDelta >  FastMath.PI) alphaDelta-=2*FastMath.PI;
        final double reductionFactor     = (FastMath.sin(alphaE) - FastMath.sin(alphaS)) / alphaDelta;


        return new ScheduledManeuver(maneuver.getModel(),
                                     maneuver.getDate(),
                                     maneuver.getDeltaV(),
                                     maneuver.getThrust(),
                                     reductionFactor * getModel().getCurrentISP(),
                                     maneuver.getTrajectory(),
                                     maneuver.isReplanned());
    }

    /** Find a node for an inclination maneuver.
     * @param start first possible date for maneuver
     * @param end last possible date for maneuver
     * @param propagator propagator for the cycle (it already takes
     * the maneuvers into account, but <em>none</em> of the {@link #getEventDetector()
     * event detector} and {@link #getStepHandler() step handler} if any)
     * @return maneuver node
     * @exception OrekitException if node cannot be found
     * @exception SkatException if node cannot be found
     */
    protected SpacecraftState findManeuverNode(final AbsoluteDate start, final AbsoluteDate end,
                                               final Propagator propagator)
        throws OrekitException, SkatException {

        for (AbsoluteDate date = start.shiftedBy(meanPeriod); date.compareTo(end) < 0; date = date.shiftedBy(meanPeriod)) {
            // find the first available node that is in eclipse for maneuver
            final double stepSize = meanPeriod / 100;
            SpacecraftState nodeState =
                    firstLatitudeCrossing(0.0, true, earth, date, end, stepSize, propagator);
            if (nodeState == null) {
                break;
            }

            Vector3D satPos = nodeState.getPVCoordinates().getPosition();
            Vector3D sunPos = sun.getPVCoordinates(nodeState.getDate(), nodeState.getFrame()).getPosition();
            if (Vector3D.dotProduct(satPos, sunPos) > 0) {
                // wrong node, it is under Sun light, select the next one
                nodeState = latitudeCrossing(0.0, earth, nodeState.getDate().shiftedBy(0.5 * meanPeriod),
                                             end, stepSize, meanPeriod / 8, propagator);
            }

            // ensure maneuvers separation requirements
            while (nodeState != null && nodeState.getDate().durationFrom(cycleStart) < firstOffset) {
                nodeState = latitudeCrossing(0.0, earth, nodeState.getDate().shiftedBy(meanPeriod),
                                             end, stepSize, meanPeriod / 8, propagator);
            }

            if (nodeState != null) {
                return nodeState;
            }

        }

        throw new SkatException(SkatMessages.NO_SUITABLE_NODE_FOR_MANEUVER, start, end);

    }

}
