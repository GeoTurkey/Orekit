/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.analysis.UnivariateRealFunction;
import org.apache.commons.math.analysis.solvers.BracketingNthOrderBrentSolver;
import org.apache.commons.math.analysis.solvers.UnivariateRealSolver;
import org.apache.commons.math.exception.NoBracketingException;
import org.apache.commons.math.geometry.euclidean.threed.Rotation;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.apache.commons.math.util.MathUtils;
import org.orekit.attitudes.Attitude;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBody;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.events.EventsLogger;
import org.orekit.propagation.events.EventsLogger.LoggedEvent;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.utils.MonitorableMonoSKData;
import eu.eumetsat.skat.utils.SkatException;

/**
 * Class for simple propagation of a station-keeping cycle.
 * <p>
 * This class performs propagation of real spacecraft state, using
 * the maneuver that have been determined in the control part of the
 * simulation.
 * </p>
 * @author Luc Maisonobe
 */
public class Propagation implements ScenarioComponent {

    /** EME 2000 Frame. */
    private final Frame eme2000;

    /** Orbit propagator. */
    private final Propagator propagator;

    /** Earth model. */
    private final BodyShape earth;

    /** Sun model. */
    private final CelestialBody sun;

    /** Output step for monitoring. */
    private final double outputstep;

    /** Monitored values. */
    private final List<MonitorableMonoSKData> monitorables;

    /** Simple constructor.
     * @param propagator propagator to use
     * @param earth Earth model
     * @param outputStep output step for monitoring (s)
     */
    public Propagation(final Propagator propagator, final BodyShape earth,
                       final CelestialBody sun, final double outputStep) {
        this.eme2000      = FramesFactory.getEME2000();
        this.propagator   = propagator;
        this.earth        = earth;
        this.sun          = sun;
        this.outputstep   = outputStep;
        this.monitorables = new ArrayList<MonitorableMonoSKData>();
    }

    /** MonitorMono a station-keeping data.
     * @param key key of station-keeping data to monitor
     */
    public void monitor(final MonitorableMonoSKData key) {
        monitorables.add(key);
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals, final AbsoluteDate target)
        throws OrekitException {

        final List<BoundedPropagator> ephemerides = new ArrayList<BoundedPropagator>();
        final List<List<LoggedEvent>> events      = new ArrayList<List<LoggedEvent>>();

        // separately propagate each spacecraft
        for (int i = 0; i < originals.length; ++i) {

            // logger used to discrete events data
            final EventsLogger logger = new EventsLogger();

            // set up the propagator with the maneuvers to perform
            propagator.clearEventsDetectors();
            for (final ImpulseManeuver maneuver : originals[i].getTheoreticalManeuvers()) {
                propagator.addEventDetector(logger.monitorDetector(maneuver));
            }
            propagator.setEphemerisMode();

            // perform propagation
            propagator.resetInitialState(originals[i].getRealStartState());

            // retrieve discrete events data
            events.add(logger.getLoggedEvents());

            // retrieve continuous data
            ephemerides.add(propagator.getGeneratedEphemeris());

        }

        // prepare temporary arrays
        ScenarioState[] updated = originals.clone();
        int[] eventIndices = new int[events.size()];

        AbsoluteDate tMin = ephemerides.get(0).getMinDate();
        AbsoluteDate tMax = ephemerides.get(0).getMaxDate();
        for (AbsoluteDate date = tMin; date.compareTo(tMax) <= 0; date = date.shiftedBy(outputstep)) {

            // check for maneuvers that have occurred
            updatePendingManeuvers(date, events, eventIndices, updated);

            // check for nodes
            updateNodes(date, ephemerides, updated);

            // monitor data
            monitorData(date, ephemerides, updated);

        }

        return updated;

    }

    /** Update the maneuvers state up to current date.
     * @param date current date
     * @param events pending discrete events
     * @param indices indices of already considered events
     * @param states states array to update
     * @exception OrekitException if a maneuver cannot be checked
     */
    private void updatePendingManeuvers(final AbsoluteDate date, final List<List<LoggedEvent>> events,
                                        final int[] indices, final ScenarioState[] states)
        throws OrekitException {
        for (int i = 0; i < states.length; ++i) {
            final int k = indices[i];
            final List<LoggedEvent> logged = events.get(i);
            while((k < logged.size()) && (logged.get(k).getState().getDate().compareTo(date) < 0)) {

                // get the maneuver event
                final SpacecraftState state  = logged.get(k).getState();
                final EventDetector detector = logged.get(k).getEventDetector();
                if (!(detector instanceof ImpulseManeuver)) {
                    // we should have only impulse maneuver events, this should never happen
                    throw SkatException.createInternalError(null);
                }
                final ImpulseManeuver maneuver = (ImpulseManeuver) detector;

                // convert velocity increment in EME2000 frame
                final Attitude attitude = state.getAttitude();
                final Rotation refToEME2000 =
                        attitude.getReferenceFrame().getTransformTo(eme2000, date).getRotation();
                final Rotation satToEME2000 = refToEME2000.applyTo(attitude.getRotation().revert());
                final Vector3D deltaV = satToEME2000.applyTo(maneuver.getDeltaVSat());

                // identify in-plane and out-of-plane maneuvers
                final double angle =
                        Vector3D.angle(state.getPVCoordinates(eme2000).getMomentum(), deltaV);

                // update the state taking the event into account
                if ((angle < FastMath.PI / 4) || (angle > 3 * FastMath.PI / 4)) {
                    // this is an out-of-plane maneuver
                    final int number = states[i].getOutOfPlane() + 1;
                    final double dv  = states[i].getOutOfPlaneDV() + maneuver.getDeltaVSat().getNorm();
                    states[i] = states[i].updateOutOfPlaneManeuvers(number, dv);
                } else {
                    // this is an in-plane maneuver
                    final int number = states[i].getInPlane() + 1;
                    final double dv  = states[i].getInPlaneDV() + maneuver.getDeltaVSat().getNorm();
                    states[i] = states[i].updateInPlaneManeuvers(number, dv);
                }

                // the event has been taken into account, prepare for next one
                ++indices[i];

            }
        }

    }

    /** Update the nodes up to current date.
     * @param date current date
     * @param ephemerides spacecrafts ephemerides
     * @param states states array to update
     * @exception OrekitException if a node cannot be computed
     */
    private void updateNodes(final AbsoluteDate date, final List<BoundedPropagator> ephemerides,
                             final ScenarioState[] states)
        throws OrekitException {
        for (int i = 0; i < states.length; ++i) {
            final double dtA = date.durationFrom(states[i].getAscendingNodeDate());
            final double dtD = date.durationFrom(states[i].getDescendingNodeDate());
            if (dtA * dtD > 0) {

                // the stored nodes do not bracket current date anymore, recompute them
                final SpacecraftState firstState = findClosestNode(ephemerides.get(i), date);
                final AbsoluteDate firstDate = firstState.getDate();
                final double period = firstState.getKeplerianPeriod();
                final double dt = (firstDate.compareTo(date) <= 0) ? 0.5 * period : -0.5 * period;
                final SpacecraftState secondState = findClosestNode(ephemerides.get(i), firstDate.shiftedBy(dt));
                final AbsoluteDate secondDate = secondState.getDate();

                if (date.durationFrom(firstDate) * date.durationFrom(secondDate) > 0) {
                    // we failed to bracket date between two nodes, this should never happen
                    throw SkatException.createInternalError(null);
                }

                final double firstSolarTime  = solarTime(firstState);
                final double secondSolarTime = solarTime(secondState);

                // update the state
                if (firstState.getPVCoordinates(eme2000).getVelocity().getZ() >= 0) {
                    // first state is an ascending node
                    states[i] = states[i].updateAscendingNodeCrossing(firstDate,   firstSolarTime);
                    states[i] = states[i].updateDescendingNodeCrossing(secondDate, secondSolarTime);
                } else {
                    // first state is a descending node
                    states[i] = states[i].updateDescendingNodeCrossing(firstDate,  firstSolarTime);
                    states[i] = states[i].updateAscendingNodeCrossing(secondDate,  secondSolarTime);
                }

            }
        }
    }

    /** Find the closest node.
     * @param ephemeride spacecraft ephemerides
     * @param date current date
     * @return spacecraft state at node
     * @exception OrekitException if ephemeride cannot be propagated at some date
     */
    private SpacecraftState findClosestNode(final BoundedPropagator ephemeride,
                                            final AbsoluteDate date)
        throws OrekitException {

        try {
            // get the orbit at curent date
            final CircularOrbit orbit =
                    (CircularOrbit) OrbitType.CIRCULAR.convertType(ephemeride.propagate(date).getOrbit());

            // rough estimate of time to closest node using Keplerian motion
            final double alphaM = MathUtils.normalizeAngle(orbit.getAlphaM(), 0);
            final double n      = orbit.getKeplerianMeanMotion();
            final double dt     = ((FastMath.abs(alphaM) <= FastMath.PI) ? -alphaM : FastMath.PI - alphaM) / n;

            // set up search range to the half orbit around node
            final double ephemerideMin  = ephemeride.getMinDate().durationFrom(date);
            final double ephemerideMax  = ephemeride.getMaxDate().durationFrom(date);
            final double halfSearchSpan = FastMath.PI / (2 * n);
            double dtMin = FastMath.max(ephemerideMin, dt - halfSearchSpan);
            double dtMax = FastMath.min(ephemerideMax, dtMin + 2 * halfSearchSpan);
            dtMin = dtMax - 2 * halfSearchSpan;

            // search the node, defined by spacecraft crossing equator in EME2000
            final UnivariateRealSolver solver = new BracketingNthOrderBrentSolver(1.0e-6, 5);
            double dtNode = solver.solve(1000, new UnivariateRealFunction() {
                /** {@inheritDoc} */
                public double value(double x) {
                    try {
                        // estimate spacecraft position with respect to equator
                        final SpacecraftState state = ephemeride.propagate(date.shiftedBy(x));
                        return state.getPVCoordinates(eme2000).getPosition().getZ();
                    } catch (OrekitException oe) {
                        throw new OrekitExceptionWrapper(oe);
                    }
                }
            }, dtMin, dtMax, dt);

            return ephemeride.propagate(date.shiftedBy(dtNode));
        } catch (NoBracketingException nbe) {
            throw new OrekitException(nbe);
        }
        
    }

    /** Compute solar time.
     * @param state spacecraft state
     * @return spacecraft local solar time (between 0 and 24 hours)
     * @throws OrekitException if sun or spacecraft positions cannot be determined
     */
    private double solarTime(final SpacecraftState state)
        throws OrekitException {
        final Vector3D spacecraftPosition = state.getPVCoordinates(eme2000).getPosition();
        final Vector3D sunPosition = sun.getPVCoordinates(state.getDate(), eme2000).getPosition();
        final double dAlpha = FastMath.PI + spacecraftPosition.getAlpha() - sunPosition.getAlpha();
        return 12.0 * MathUtils.normalizeAngle(dAlpha, FastMath.PI) / FastMath.PI;
    }

    /** MonitorMono the station-keeping data up to current date.
     * @param date current date
     * @param ephemerides spacecrafts ephemerides
     * @param states states array to update
     * @exception OrekitException if a node cannot be computed
     */
    private void monitorData(final AbsoluteDate date, final List<BoundedPropagator> ephemerides,
                             final ScenarioState[] states)
        throws OrekitException {

        // update the real states
        for (int i = 0; i < states.length; ++i) {
            final SpacecraftState state = ephemerides.get(i).propagate(date);
            states[i] = states[i].updateRealEndState(state);
        }

        // perform monitoring
        for (final MonitorableMonoSKData monitorable : monitorables) {
            monitorable.update(states, earth);
        }

    }

}
