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
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinates;

import eu.eumetsat.skat.scenario.ScenarioState;

/** Enumerate representing time-dependent values that can be monitored.
 * <p>
 * Monitoring time-dependent values allows storing their evolution in
 * csv files or displaying them as graphical curves.
 * </p>
 * 
 * @see Monitor
 * @author Luc Maisonobe
 */
public enum MonitorableSKData implements Monitorable {

    IN_PLANE_MANEUVER_NUMBER(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                data[i] = states[i].getInPlane();
            }
        }
        
    },

    IN_PLANE_MANEUVER_TOTAL_DV(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                data[i] = states[i].getInPlaneDV();
            }
        }
        
    },

    OUT_OF_PLANE_MANEUVER_NUMBER(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                data[i] = states[i].getOutOfPlane();
            }
        }
        
    },

    OUT_OF_PLANE_MANEUVER_TOTAL_DV(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                data[i] = states[i].getOutOfPlaneDV();
            }
        }
        
    },

    PROPELLANT_MASS_CONSUMPTION(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                data[i] = states[i].getMassConsumption();
            }
        }
        
    },

    SPACECRAFT_MASS(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                data[i] = states[i].getRealState().getMass();
            }
        }
        
    },

    POSITION_EME2000(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data)
            throws OrekitException {
            final Frame eme2000 = FramesFactory.getEME2000();
            for (int i = 0; i < states.length; ++i) {
                final PVCoordinates pv = states[i].getRealState().getPVCoordinates(eme2000);
                final Vector3D position = pv.getPosition();
                data[3 * i]     = position.getX();
                data[3 * i + 1] = position.getY();
                data[3 * i + 2] = position.getZ();
            }
        }
        
    },

    VELOCITY_EME2000(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data)
            throws OrekitException {
            final Frame eme2000 = FramesFactory.getEME2000();
            for (int i = 0; i < states.length; ++i) {
                final PVCoordinates pv = states[i].getRealState().getPVCoordinates(eme2000);
                final Vector3D velocity = pv.getVelocity();
                data[3 * i]     = velocity.getX();
                data[3 * i + 1] = velocity.getY();
                data[3 * i + 2] = velocity.getZ();
            }
        }
        
    },

    POSITION_ITRF(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data)
            throws OrekitException {
            final Frame itrf = FramesFactory.getITRF2008();
            for (int i = 0; i < states.length; ++i) {
                final PVCoordinates pv = states[i].getRealState().getPVCoordinates(itrf);
                final Vector3D position = pv.getPosition();
                data[3 * i]     = position.getX();
                data[3 * i + 1] = position.getY();
                data[3 * i + 2] = position.getZ();
            }
        }
        
    },

    VELOCITY_ITRF(3) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data)
            throws OrekitException {
            final Frame itrf = FramesFactory.getITRF2008();
            for (int i = 0; i < states.length; ++i) {
                final PVCoordinates pv = states[i].getRealState().getPVCoordinates(itrf);
                final Vector3D velocity = pv.getVelocity();
                data[3 * i]     = velocity.getX();
                data[3 * i + 1] = velocity.getY();
                data[3 * i + 2] = velocity.getZ();
            }
        }
        
    },

    LATITUDE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data)
                throws OrekitException {
            final Frame itrf = FramesFactory.getITRF2008();
            for (int i = 0; i < states.length; ++i) {
                final PVCoordinates pv   = states[i].getRealState().getPVCoordinates(itrf);
                final AbsoluteDate  date = states[i].getRealState().getDate();
                final GeodeticPoint gp   = earth.transform(pv.getPosition(), itrf, date);
                data[i] = gp.getLatitude();
            }
        }

    },

    LONGITUDE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data)
                throws OrekitException {
            final Frame itrf = FramesFactory.getITRF2008();
            for (int i = 0; i < states.length; ++i) {
                final PVCoordinates pv   = states[i].getRealState().getPVCoordinates(itrf);
                final AbsoluteDate  date = states[i].getRealState().getDate();
                final GeodeticPoint gp   = earth.transform(pv.getPosition(), itrf, date);
                data[i] = gp.getLongitude();
            }
        }
        
    },

    ALTITUDE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data)
                throws OrekitException {
            final Frame itrf = FramesFactory.getITRF2008();
            for (int i = 0; i < states.length; ++i) {
                final PVCoordinates pv   = states[i].getRealState().getPVCoordinates(itrf);
                final AbsoluteDate  date = states[i].getRealState().getDate();
                final GeodeticPoint gp   = earth.transform(pv.getPosition(), itrf, date);
                data[i] = gp.getAltitude();
            }
        }
        
    },

    SEMI_MAJOR_AXIS(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                final KeplerianOrbit orbit =
                        (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(states[i].getRealState().getOrbit());
                data[i] = orbit.getA();
            }
        }
        
    },

    ECCENTRICITY(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                final KeplerianOrbit orbit =
                        (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(states[i].getRealState().getOrbit());
                data[i] = orbit.getE();
            }
        }
        
    },

    INCLINATION(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                final KeplerianOrbit orbit =
                        (KeplerianOrbit) OrbitType.KEPLERIAN.convertType(states[i].getRealState().getOrbit());
                data[i] = orbit.getI();
            }
        }
        
    },

    CIRCULAR_ECCENTRICITY_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                final CircularOrbit orbit =
                        (CircularOrbit) OrbitType.CIRCULAR.convertType(states[i].getRealState().getOrbit());
                data[2 * i]     = orbit.getCircularEx();
                data[2 * i + 1] = orbit.getCircularEy();
            }
        }
        
    },

    EQUINOCTIAL_ECCENTRICITY_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                final EquinoctialOrbit orbit =
                        (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(states[i].getRealState().getOrbit());
                data[2 * i]     = orbit.getEquinoctialEx();
                data[2 * i + 1] = orbit.getEquinoctialEy();
            }
        }
        
    },

    EQUINOCTIAL_INCLINATION_VECTOR(2) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                final EquinoctialOrbit orbit =
                        (EquinoctialOrbit) OrbitType.EQUINOCTIAL.convertType(states[i].getRealState().getOrbit());
                data[2 * i]     = orbit.getHx();
                data[2 * i + 1] = orbit.getHy();
            }
        }
        
    },

    SOLAR_TIME_AT_ASCENDING_NODE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            // TODO Auto-generated method stub
        }
        
    },

    SOLAR_TIME_AT_DESCENDING_NODE(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            // TODO Auto-generated method stub
        }
        
    },

    CYCLES_NUMBER(1) {

        /** {@inheritDoc} */
        @Override
        protected void extractData(final ScenarioState[] states, BodyShape earth, double[] data) {
            for (int i = 0; i < states.length; ++i) {
                data[i] = states[i].getCyclesNumber();
            }
        }
        
    };

    /** Current date. */
    private AbsoluteDate date;

    /** Current value. */
    private final double[] value;

    /** Monitors interested in monitoring this value. */
    private final Set<Monitor> monitors;

    /** Simple constructor.
     * @param name name of the time-dependent value
     * @param dimension expected dimension of the value
     */
    private MonitorableSKData(final int dimension) {
        this.date  = AbsoluteDate.PAST_INFINITY;
        this.value = new double[dimension];
        Arrays.fill(value, Double.NaN);
        this.monitors = new HashSet<Monitor>();
    }

    /** {@inheritDoc} */
    public void register(Monitor monitor) {
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
    public double[] getValue() {
        return value.clone();
    }

    /** Update the current date and value, and notifies all monitors.
     * @param states states of all spacecrafts
     * @param earth Earth model
     * @exception OrekitException if data cannot be computed
     */
    public void update(final ScenarioState[] states, final BodyShape earth)
        throws OrekitException {

        date = states[0].getRealState().getDate();
        extractData(states, earth, value);

        // notifies monitors
        for (final Monitor monitor : monitors) {
            monitor.valueChanged(this);
        }

    }

    /** Extract the monitored data from the states.
     * @param states states of all spacecrafts
     * @param earth Earth model
     * @param data placeholder where to put the extracted data
     * @exception OrekitException if data cannot be computed
     */
    protected abstract void extractData(ScenarioState[] states, BodyShape earth,
                                        double[] data)
        throws OrekitException;

}
