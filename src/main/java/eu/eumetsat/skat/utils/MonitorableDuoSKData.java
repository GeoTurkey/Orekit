/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.BodyShape;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.time.AbsoluteDate;

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

    DISTANCE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data)
            throws OrekitException {
            final Vector3D otherP =
                    otherState.getRealStartState().getPVCoordinates(FramesFactory.getEME2000()).getPosition();
            final Vector3D referenceP =
                    referenceState.getRealStartState().getPVCoordinates(FramesFactory.getEME2000()).getPosition();
            data[0] = otherP.distance(referenceP);
        }

    },

    ANGULAR_SEPARATION_FROM_EARTH_CENTER(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data)
            throws OrekitException {
            final Vector3D otherP =
                    otherState.getRealStartState().getPVCoordinates().getPosition();
            final Vector3D referenceP =
                    referenceState.getRealStartState().getPVCoordinates().getPosition();
            data[0] = Vector3D.angle(referenceP, otherP);
        }

    },

    ANGULAR_SEPARATION_FROM_REFERENCE_LOCATION(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data)
            throws OrekitException {
            final Vector3D otherP =
                    otherState.getRealStartState().getPVCoordinates(groundLocation).getPosition();
            final Vector3D referenceP =
                    referenceState.getRealStartState().getPVCoordinates(groundLocation).getPosition();
            data[0] = Vector3D.angle(referenceP, otherP);
        }

    },

    RELATIVE_POSITION_EME2000(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data)
            throws OrekitException {
            final Vector3D otherP =
                    otherState.getRealStartState().getPVCoordinates(FramesFactory.getEME2000()).getPosition();
            final Vector3D referenceP =
                    referenceState.getRealStartState().getPVCoordinates(FramesFactory.getEME2000()).getPosition();
            data[0] = otherP.getX() - referenceP.getX();
            data[1] = otherP.getY() - referenceP.getY();
            data[2] = otherP.getZ() - referenceP.getZ();
        }

    },

    RELATIVE_VELOCITY_EME2000(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data)
            throws OrekitException {
            final Vector3D otherV =
                    otherState.getRealStartState().getPVCoordinates(FramesFactory.getEME2000()).getVelocity();
            final Vector3D referenceV =
                    referenceState.getRealStartState().getPVCoordinates(FramesFactory.getEME2000()).getVelocity();
            data[0] = otherV.getX() - referenceV.getX();
            data[1] = otherV.getY() - referenceV.getY();
            data[2] = otherV.getZ() - referenceV.getZ();
        }

    },

    RELATIVE_POSITION_ITRF(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data)
            throws OrekitException {
            final Vector3D otherP =
                    otherState.getRealStartState().getPVCoordinates(FramesFactory.getITRF2008()).getPosition();
            final Vector3D referenceP =
                    referenceState.getRealStartState().getPVCoordinates(FramesFactory.getITRF2008()).getPosition();
            data[0] = otherP.getX() - referenceP.getX();
            data[1] = otherP.getY() - referenceP.getY();
            data[2] = otherP.getZ() - referenceP.getZ();
        }

    },

    RELATIVE_VELOCITY_ITRF(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data)
            throws OrekitException {
            final Vector3D otherV =
                    otherState.getRealStartState().getPVCoordinates(FramesFactory.getITRF2008()).getVelocity();
            final Vector3D referenceV =
                    referenceState.getRealStartState().getPVCoordinates(FramesFactory.getITRF2008()).getVelocity();
            data[0] = otherV.getX() - referenceV.getX();
            data[1] = otherV.getY() - referenceV.getY();
            data[2] = otherV.getZ() - referenceV.getZ();
        }

    },

    SEMI_MAJOR_AXIS_DIFFERENCE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data) {
            final KeplerianOrbit otherOrbit =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(otherState.getRealStartState().getOrbit());
            final KeplerianOrbit referenceOrbit =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(referenceState.getRealStartState().getOrbit());
            data[0] = otherOrbit.getA() - referenceOrbit.getA();
        }

    },

    ECCENTRICITY_DIFFERENCE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data) {
            final KeplerianOrbit otherOrbit =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(otherState.getRealStartState().getOrbit());
            final KeplerianOrbit referenceOrbit =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(referenceState.getRealStartState().getOrbit());
            data[0] = otherOrbit.getE() - referenceOrbit.getE();
        }

    },

    INCLINATION_DIFFERENCE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data) {
            final KeplerianOrbit otherOrbit =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(otherState.getRealStartState().getOrbit());
            final KeplerianOrbit referenceOrbit =
                    (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(referenceState.getRealStartState().getOrbit());
            data[0] = otherOrbit.getI() - referenceOrbit.getI();
        }

    },

    RELATIVE_CIRCULAR_ECCENTRICITY_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data) {
            final CircularOrbit otherOrbit =
                    (CircularOrbit) OrbitType.CIRCULAR.convertType(otherState.getRealStartState().getOrbit());
            final CircularOrbit referenceOrbit =
                    (CircularOrbit) OrbitType.CIRCULAR.convertType(referenceState.getRealStartState().getOrbit());
            data[0] = otherOrbit.getCircularEx() - referenceOrbit.getCircularEx();
            data[1] = otherOrbit.getCircularEy() - referenceOrbit.getCircularEy();
        }

    },

    RELATIVE_EQUINOCTIAL_ECCENTRICITY_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data) {
            final EquinoctialOrbit otherOrbit =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(otherState.getRealStartState().getOrbit());
            final EquinoctialOrbit referenceOrbit =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(referenceState.getRealStartState().getOrbit());
            data[0] = otherOrbit.getEquinoctialEx() - referenceOrbit.getEquinoctialEx();
            data[1] = otherOrbit.getEquinoctialEy() - referenceOrbit.getEquinoctialEy();
        }

    },

    RELATIVE_EQUINOCTIAL_INCLINATION_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState referenceState, final ScenarioState otherState,
                                   final BodyShape earth, TopocentricFrame groundLocation, double[] data) {
            final EquinoctialOrbit otherOrbit =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(otherState.getRealStartState().getOrbit());
            final EquinoctialOrbit referenceOrbit =
                    (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(referenceState.getRealStartState().getOrbit());
            data[0] = otherOrbit.getHx() - referenceOrbit.getHx();
            data[1] = otherOrbit.getHy() - referenceOrbit.getHy();
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
        this.monitors  = new HashSet<MonitorDuo>();
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
     * @param groundLocation location of reference ground point
     * @exception OrekitException if data cannot be computed
     */
    public void update(final ScenarioState[] states, final BodyShape earth,
                       final TopocentricFrame groundLocation)
            throws OrekitException {

        date = states[0].getRealStartState().getDate();
        for (int i = 0; i < states.length; ++i) {
            for (int j = 0; j < states.length; ++j) {
                if (i != j) {
                    extractData(states[i], states[j], earth, groundLocation, value[i][j]);
                }
            }
        }

        // notifies monitors
        for (final MonitorDuo monitor : monitors) {
            monitor.valueChanged(this);
        }

    }

    /** Extract the monitored data from the states.
     * @param referenceState state of the reference spacecraft
     * @param otherState state of the other spacecraft
     * @param earth Earth model
     * @param groundLocation location of reference ground point
     * @param data placeholder where to put the extracted data
     * @exception OrekitException if data cannot be computed
     */
    protected abstract void extractData(ScenarioState referenceState, ScenarioState otherState,
                                        BodyShape earth, TopocentricFrame groundLocation,
                                        double[] data)
        throws OrekitException;

}
