/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinates;

import eu.eumetsat.skat.scenario.ScenarioState;

/** Enumerate representing time-dependent values from a single spacecraft
 * that can be monitored.
 * <p>
 * Monitoring time-dependent values allows storing their evolution in
 * csv files or displaying them as graphical curves.
 * </p>
 * 
 * @see MonitorMono
 * @author Luc Maisonobe
 */
public enum MonitorableMonoSKData implements MonitorableMono {

    IN_PLANE_MANEUVER_NUMBER(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getInPlaneManeuvers();
        }

    },

    IN_PLANE_MANEUVER_CYCLE_DV(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getInPlaneCycleDV();
        }

    },

    IN_PLANE_MANEUVER_TOTAL_DV(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getInPlaneTotalDV();
        }

    },

    OUT_OF_PLANE_MANEUVER_NUMBER(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getOutOfPlaneManeuvers();
        }

    },

    OUT_OF_PLANE_MANEUVER_CYCLE_DV(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getOutOfPlaneCycleDV();
        }

    },

    OUT_OF_PLANE_MANEUVER_TOTAL_DV(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getOutOfPlaneTotalDV();
        }

    },

    SPACECRAFT_MASS(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getRealStartState().getMass();
        }

    },

    POSITION_EME2000(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            final Vector3D position = getPVCoordinates(state, FramesFactory.getEME2000()).getPosition();
            data[0] = position.getX();
            data[1] = position.getY();
            data[2] = position.getZ();
        }

    },

    VELOCITY_EME2000(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            final Vector3D velocity = getPVCoordinates(state, FramesFactory.getEME2000()).getVelocity();
            data[0] = velocity.getX();
            data[1] = velocity.getY();
            data[2] = velocity.getZ();
        }

    },

    POSITION_ITRF(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
           throws OrekitException {
            final Vector3D position = getPVCoordinates(state, FramesFactory.getITRF2008()).getPosition();
            data[0] = position.getX();
            data[1] = position.getY();
            data[2] = position.getZ();
        }

    },

    VELOCITY_ITRF(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            final Vector3D velocity = getPVCoordinates(state, FramesFactory.getITRF2008()).getVelocity();
            data[0] = velocity.getX();
            data[1] = velocity.getY();
            data[2] = velocity.getZ();
        }

    },

    LATITUDE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            final Vector3D position  = getPVCoordinates(state, FramesFactory.getITRF2008()).getPosition();
            final AbsoluteDate  date = state.getRealStartState().getDate();
            final GeodeticPoint gp   = earth.transform(position, FramesFactory.getITRF2008(), date);
            data[0] = gp.getLatitude();
        }

    },

    LONGITUDE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            final Vector3D position  = getPVCoordinates(state, FramesFactory.getITRF2008()).getPosition();
            final AbsoluteDate  date = state.getRealStartState().getDate();
            final GeodeticPoint gp   = earth.transform(position, FramesFactory.getITRF2008(), date);
            data[0] = gp.getLongitude();
        }

    },

    ALTITUDE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            final Vector3D position  = getPVCoordinates(state, FramesFactory.getITRF2008()).getPosition();
            final AbsoluteDate  date = state.getRealStartState().getDate();
            final GeodeticPoint gp   = earth.transform(position, FramesFactory.getITRF2008(), date);
            data[0] = gp.getAltitude();
        }

    },

    SEMI_MAJOR_AXIS(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            data[0] = getKeplerianOrbit(state).getA();
        }

    },

    ECCENTRICITY(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            data[0] = getKeplerianOrbit(state).getE();
        }

    },

    INCLINATION(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
                data[0] = getKeplerianOrbit(state).getI();
        }

    },

    CIRCULAR_ECCENTRICITY_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            final CircularOrbit orbit = getCircularOrbit(state);
            data[0] = orbit.getCircularEx();
            data[1] = orbit.getCircularEy();
        }

    },

    EQUINOCTIAL_ECCENTRICITY_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            final EquinoctialOrbit orbit = getEquinoctialOrbit(state);
            data[0] = orbit.getEquinoctialEx();
            data[1] = orbit.getEquinoctialEy();
        }

    },

    EQUINOCTIAL_INCLINATION_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            final EquinoctialOrbit orbit = getEquinoctialOrbit(state);
            data[0] = orbit.getHx();
            data[1] = orbit.getHy();
        }

    },

    SOLAR_TIME_AT_ASCENDING_NODE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getAscendingNodesSolarTime();
        }

    },

    SOLAR_TIME_AT_DESCENDING_NODE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getDescendingNodesSolarTime();
        }

    },

    CYCLES_NUMBER(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getCyclesNumber();
        }

    };

    /** Dimension of the value. */
    private final int dimension;

    /** Current date. */
    private AbsoluteDate date;

    /** Current value. */
    private double[][] value;

    /** Monitors interested in monitoring this value. */
    private final Set<MonitorMono> monitors;

    /** Simple constructor.
     * @param name name of the time-dependent value
     * @param dimension expected dimension of the value
     */
    private MonitorableMonoSKData(final int dimension) {
        this.dimension = dimension;
        this.date      = AbsoluteDate.PAST_INFINITY;
        this.value     = null;
        this.monitors = new HashSet<MonitorMono>();
    }

    /** {@inheritDoc} */
    public void register(final int nbSpacecrafts, final MonitorMono monitor) {

        // lazy memory allocation
        if (value == null) {
            value = new double[nbSpacecrafts][dimension];
            for (double[] row : value) {
                Arrays.fill(row, Double.NaN);
            }
        }

        monitors.add(monitor);
        monitor.startMonitoring(this);

    }

    /** {@inheritDoc} */
    public String getName() {
        return toString().toLowerCase().replaceAll("_", " ");
    }

    /** {@inheritDoc} */
    public AbsoluteDate getDate() {
        return date;
    }

    /** {@inheritDoc} */
    public double[] getValue(final int spacecraftIdx) {
        return value[spacecraftIdx].clone();
    }

    /** Get the current spacecraft state.
     * @param state scenario state
     * @return current spacecraft state
     * @exception OrekitException if state cannot be extracted from performed ephemeris
     */
    private SpacecraftState getSpacecraftState(final ScenarioState state) throws OrekitException {
        return state.getPerformedEphemeris().propagate(getDate());
    }

    /** Get the current coordinates.
     * @param state scenario state
     * @param frame frame in which coordinates are requested
     * @return current coordinates in specified frame
     * @exception OrekitException if state cannot be extracted from performed ephemeris
     */
    protected PVCoordinates getPVCoordinates(final ScenarioState state, final Frame frame)
        throws OrekitException {
        return getSpacecraftState(state).getPVCoordinates(frame);
    }

    /** Get the current Keplerian orbit.
     * @param state scenario state
     * @return current Keplerian orbit
     * @exception OrekitException if state cannot be extracted from performed ephemeris
     */
    protected KeplerianOrbit getKeplerianOrbit(final ScenarioState state)
        throws OrekitException {
        return (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(getSpacecraftState(state).getOrbit());
    }

    /** Get the current circular orbit.
     * @param state scenario state
     * @return current circular orbit
     * @exception OrekitException if state cannot be extracted from performed ephemeris
     */
    protected CircularOrbit getCircularOrbit(final ScenarioState state)
        throws OrekitException {
        return (CircularOrbit) OrbitType.CIRCULAR.convertType(getSpacecraftState(state).getOrbit());
    }

    /** Get the current equinoctial orbit.
     * @param state scenario state
     * @return current equinoctial orbit
     * @exception OrekitException if state cannot be extracted from performed ephemeris
     */
    protected EquinoctialOrbit getEquinoctialOrbit(final ScenarioState state)
        throws OrekitException {
        return (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(getSpacecraftState(state).getOrbit());
    }

    /** Update the current date and value, and notifies all monitors.
     * @param date current date
     * @param states states of all spacecrafts
     * @param earth Earth model
     * @exception OrekitException if data cannot be computed
     */
    public void update(final AbsoluteDate date, final ScenarioState[] states, final BodyShape earth)
            throws OrekitException {

        this.date = date;
        for (int i = 0; i < states.length; ++i) {
            extractData(states[i], earth, value[i]);
        }

        // notifies monitors
        for (final MonitorMono monitor : monitors) {
            monitor.valueChanged(this);
        }

    }

    /** Extract the monitored data from the states.
     * @param state state of one spacecraft
     * @param earth Earth model
     * @param data placeholder where to put the extracted data
     * @exception OrekitException if data cannot be computed
     */
    protected abstract void extractData(ScenarioState state, BodyShape earth, double[] data)
        throws OrekitException;

}
