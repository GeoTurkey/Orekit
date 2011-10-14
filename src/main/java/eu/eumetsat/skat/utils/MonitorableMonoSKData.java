/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.OrbitType;
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
            data[0] = state.getInPlane();
        }

    },

    IN_PLANE_MANEUVER_TOTAL_DV(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getInPlaneDV();
        }

    },

    OUT_OF_PLANE_MANEUVER_NUMBER(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getOutOfPlane();
        }

    },

    OUT_OF_PLANE_MANEUVER_TOTAL_DV(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getOutOfPlaneDV();
        }

    },

    PROPELLANT_MASS_CONSUMPTION(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            data[0] = state.getMassConsumption();
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
            final PVCoordinates pv = state.getRealStartState().getPVCoordinates(FramesFactory.getEME2000());
            final Vector3D position = pv.getPosition();
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
            final PVCoordinates pv = state.getRealStartState().getPVCoordinates(FramesFactory.getEME2000());
            final Vector3D velocity = pv.getVelocity();
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
            final PVCoordinates pv = state.getRealStartState().getPVCoordinates(FramesFactory.getITRF2008());
            final Vector3D position = pv.getPosition();
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
            final PVCoordinates pv = state.getRealStartState().getPVCoordinates(FramesFactory.getITRF2008());
            final Vector3D velocity = pv.getVelocity();
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
            final PVCoordinates pv   = state.getRealStartState().getPVCoordinates(FramesFactory.getITRF2008());
            final AbsoluteDate  date = state.getRealStartState().getDate();
            final GeodeticPoint gp   = earth.transform(pv.getPosition(), FramesFactory.getITRF2008(), date);
            data[0] = gp.getLatitude();
        }

    },

    LONGITUDE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            final PVCoordinates pv   = state.getRealStartState().getPVCoordinates(FramesFactory.getITRF2008());
            final AbsoluteDate  date = state.getRealStartState().getDate();
            final GeodeticPoint gp   = earth.transform(pv.getPosition(), FramesFactory.getITRF2008(), date);
            data[0] = gp.getLongitude();
        }

    },

    ALTITUDE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data)
            throws OrekitException {
            final PVCoordinates pv   = state.getRealStartState().getPVCoordinates(FramesFactory.getITRF2008());
            final AbsoluteDate  date = state.getRealStartState().getDate();
            final GeodeticPoint gp   = earth.transform(pv.getPosition(), FramesFactory.getITRF2008(), date);
            data[0] = gp.getAltitude();
        }

    },

    SEMI_MAJOR_AXIS(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            final KeplerianOrbit orbit =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(state.getRealStartState().getOrbit());
            data[0] = orbit.getA();
        }

    },

    ECCENTRICITY(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            final KeplerianOrbit orbit =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(state.getRealStartState().getOrbit());
            data[0] = orbit.getE();
        }

    },

    INCLINATION(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            final KeplerianOrbit orbit =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(state.getRealStartState().getOrbit());
            data[0] = orbit.getI();
        }

    },

    CIRCULAR_ECCENTRICITY_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            final CircularOrbit orbit =
                    (CircularOrbit) OrbitType.CIRCULAR.convertType(state.getRealStartState().getOrbit());
            data[0] = orbit.getCircularEx();
            data[1] = orbit.getCircularEy();
        }

    },

    EQUINOCTIAL_ECCENTRICITY_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            final EquinoctialOrbit orbit =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state.getRealStartState().getOrbit());
            data[0] = orbit.getEquinoctialEx();
            data[1] = orbit.getEquinoctialEy();
        }

    },

    EQUINOCTIAL_INCLINATION_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state, BodyShape earth, double[] data) {
            final EquinoctialOrbit orbit =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state.getRealStartState().getOrbit());
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
    public void register(final int nbSpacrafts, final MonitorMono monitor) {

        // lazy memory allocation
        if (value == null) {
            value = new double[nbSpacrafts][dimension];
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

    /** Update the current date and value, and notifies all monitors.
     * @param states states of all spacecrafts
     * @param earth Earth model
     * @exception OrekitException if data cannot be computed
     */
    public void update(final ScenarioState[] states, final BodyShape earth)
            throws OrekitException {

        date = states[0].getRealStartState().getDate();
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
