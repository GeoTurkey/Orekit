/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.BodyShape;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinates;

import eu.eumetsat.skat.scenario.ScenarioState;

/** Enumerate representing time-dependent values from two spacecrafts
 * that can be monitored.
 * <p>
 * Monitoring time-dependent values allows storing their evolution in
 * csv files or displaying them as graphical curves.
 * </p>
 * 
 * @see MonitorDuo
 * @author Luc Maisonobe
 */
public enum MonitorableDuoSKData implements MonitorableDuo {

    DELTA_POSITION_EME2000(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state1, final ScenarioState state2,
                                   final BodyShape earth, double[] data)
            throws OrekitException {
            final PVCoordinates pv1 = state1.getRealState().getPVCoordinates(FramesFactory.getEME2000());
            final Vector3D position1 = pv1.getPosition();
            final PVCoordinates pv2 = state2.getRealState().getPVCoordinates(FramesFactory.getEME2000());
            final Vector3D position2 = pv2.getPosition();
            data[0] = position1.getX() - position2.getX();
            data[1] = position1.getY() - position2.getY();
            data[2] = position1.getZ() - position2.getZ();
        }

    },

    DELTA_VELOCITY_EME2000(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state1, final ScenarioState state2,
                                   final BodyShape earth, double[] data)
            throws OrekitException {
            final PVCoordinates pv1 = state1.getRealState().getPVCoordinates(FramesFactory.getEME2000());
            final Vector3D velocity1 = pv1.getVelocity();
            final PVCoordinates pv2 = state2.getRealState().getPVCoordinates(FramesFactory.getEME2000());
            final Vector3D velocity2 = pv2.getVelocity();
            data[0] = velocity1.getX() - velocity2.getX();
            data[1] = velocity1.getY() - velocity2.getY();
            data[2] = velocity1.getZ() - velocity2.getZ();
        }

    },

    DELTA_POSITION_ITRF(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state1, final ScenarioState state2,
                                   final BodyShape earth, double[] data)
            throws OrekitException {
            final PVCoordinates pv1 = state1.getRealState().getPVCoordinates(FramesFactory.getITRF2008());
            final Vector3D position1 = pv1.getPosition();
            final PVCoordinates pv2 = state2.getRealState().getPVCoordinates(FramesFactory.getITRF2008());
            final Vector3D position2 = pv2.getPosition();
            data[0] = position1.getX() - position2.getX();
            data[1] = position1.getY() - position2.getY();
            data[2] = position1.getZ() - position2.getZ();
        }

    },

    DELTA_VELOCITY_ITRF(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state1, final ScenarioState state2,
                                   final BodyShape earth, double[] data)
            throws OrekitException {
            final PVCoordinates pv1 = state1.getRealState().getPVCoordinates(FramesFactory.getITRF2008());
            final Vector3D velocity1 = pv1.getVelocity();
            final PVCoordinates pv2 = state2.getRealState().getPVCoordinates(FramesFactory.getITRF2008());
            final Vector3D velocity2 = pv2.getVelocity();
            data[0] = velocity1.getX() - velocity2.getX();
            data[1] = velocity1.getY() - velocity2.getY();
            data[2] = velocity1.getZ() - velocity2.getZ();
        }

    },

    DELTA_SEMI_MAJOR_AXIS(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state1, final ScenarioState state2,
                                   final BodyShape earth, double[] data) {
            final KeplerianOrbit orbit1 =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(state1.getRealState().getOrbit());
            final KeplerianOrbit orbit2 =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(state2.getRealState().getOrbit());
            data[0] = orbit1.getA() - orbit2.getA();
        }

    },

    DELTA_ECCENTRICITY(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state1, final ScenarioState state2,
                                   final BodyShape earth, double[] data) {
            final KeplerianOrbit orbit1 =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(state1.getRealState().getOrbit());
            final KeplerianOrbit orbit2 =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(state2.getRealState().getOrbit());
            data[0] = orbit1.getE() - orbit2.getE();
        }

    },

    DELTA_INCLINATION(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state1, final ScenarioState state2,
                                   final BodyShape earth, double[] data) {
            final KeplerianOrbit orbit1 =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(state1.getRealState().getOrbit());
            final KeplerianOrbit orbit2 =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(state2.getRealState().getOrbit());
            data[0] = orbit1.getI() - orbit2.getI();
        }

    },

    DELTA_CIRCULAR_ECCENTRICITY_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state1, final ScenarioState state2,
                                   final BodyShape earth, double[] data) {
            final CircularOrbit orbit1 =
                    (CircularOrbit) OrbitType.CIRCULAR.convertType(state1.getRealState().getOrbit());
            final CircularOrbit orbit2 =
                    (CircularOrbit) OrbitType.CIRCULAR.convertType(state2.getRealState().getOrbit());
            data[0] = orbit1.getCircularEx() - orbit2.getCircularEx();
            data[1] = orbit1.getCircularEy() - orbit2.getCircularEy();
        }

    },

    DELTA_EQUINOCTIAL_ECCENTRICITY_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state1, final ScenarioState state2,
                                   final BodyShape earth, double[] data) {
            final EquinoctialOrbit orbit1 =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state1.getRealState().getOrbit());
            final EquinoctialOrbit orbit2 =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state2.getRealState().getOrbit());
            data[0] = orbit1.getEquinoctialEx() - orbit2.getEquinoctialEx();
            data[1] = orbit1.getEquinoctialEy() - orbit2.getEquinoctialEy();
        }

    },

    DELTA_EQUINOCTIAL_INCLINATION_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState state1, final ScenarioState state2,
                                   final BodyShape earth, double[] data) {
            final EquinoctialOrbit orbit1 =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state1.getRealState().getOrbit());
            final EquinoctialOrbit orbit2 =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(state2.getRealState().getOrbit());
            data[0] = orbit1.getHx() - orbit2.getHx();
            data[1] = orbit1.getHy() - orbit2.getHy();
        }

    };

    /** Dimension of the value. */
    private final int dimension;

    /** Current date. */
    private AbsoluteDate date;

    /** Current value. */
    private double[][][] value;

    /** Monitors interested in monitoring this value. */
    private final Set<MonitorDuo> monitors;

    /** Simple constructor.
     * @param name name of the time-dependent value
     * @param dimension expected dimension of the value
     */
    private MonitorableDuoSKData(final int dimension) {
        this.dimension = dimension;
        this.date      = AbsoluteDate.PAST_INFINITY;
        this.value     = null;
        this.monitors = new HashSet<MonitorDuo>();
    }

    /** {@inheritDoc} */
    public void register(final int nbSpacrafts, MonitorDuo monitor) {

        // lazy memory allocation
        if (value == null) {
            value = new double[nbSpacrafts][nbSpacrafts][dimension];
            for (final double[][] array : value) {
                for (final double[] row : array) {
                    Arrays.fill(row, Double.NaN);
                }
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
    public double[] getValue(int spacecraftIdx1, int spacecraftIdx2) {
        return value[spacecraftIdx1][spacecraftIdx2].clone();
    }

    /** Update the current date and value, and notifies all monitors.
     * @param states states of all spacecrafts
     * @param earth Earth model
     * @exception OrekitException if data cannot be computed
     */
    public void update(final ScenarioState[] states, final BodyShape earth)
            throws OrekitException {

        date = states[0].getRealState().getDate();
        for (int i = 0; i < states.length; ++i) {
            for (int j = 0; j < states.length; ++j) {
                if (i != j) {
                    extractData(states[i], states[j], earth, value[i][j]);
                }
            }
        }

        // notifies monitors
        for (final MonitorDuo monitor : monitors) {
            monitor.valueChanged(this);
        }

    }

    /** Extract the monitored data from the states.
     * @param state state of one spacecraft
     * @param earth Earth model
     * @param data placeholder where to put the extracted data
     * @exception OrekitException if data cannot be computed
     */
    protected abstract void extractData(ScenarioState state1, ScenarioState state2,
                                        BodyShape earth, double[] data)
        throws OrekitException;

}
