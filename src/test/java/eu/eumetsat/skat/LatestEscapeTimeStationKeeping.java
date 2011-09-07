/* Copyright 2011 Eumetsat */

package eu.eumetsat.skat;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import java.net.URISyntaxException;
import java.net.URL;
import java.text.ParseException;
import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.ode.nonstiff.AdaptiveStepsizeIntegrator;
import org.apache.commons.math.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math.optimization.GoalType;
import org.apache.commons.math.optimization.univariate.BrentOptimizer;
import org.apache.commons.math.optimization.univariate.UnivariateRealPointValuePair;
import org.apache.commons.math.util.FastMath;
import org.orekit.attitudes.LofOffset;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.forces.ForceModel;
import org.orekit.forces.gravity.CunninghamAttractionModel;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.PotentialCoefficientsProvider;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.LOFType;
import org.orekit.frames.Predefined;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.propagation.sampling.OrekitFixedStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;

/** Feasibility check application for blind-optimization based station keeping.
 * <p>
 * This feasibility check is based on longitude window escape time. It attempts
 * to compute a maneuver that push the escape time at the latest possible date,
 * which is equivalent to stay in the station keeping window as long as possible.
 * </p>
 */
public class LatestEscapeTimeStationKeeping {

    /** Initial orbit. */
    private final SpacecraftState initialState;

    /** Earth model. */
    private final BodyShape earth;

    /** Orbit propagator. */
    private final Propagator propagator;

    /** InertialFrame. */
    private final Frame inertialFrame;

    /** Start date. */
    private final AbsoluteDate startDate;

    /** Number of cycles. */
    private final int cyclesNumber;

    /** Maximal cycle duration. */
    private final double cycleDuration;

    /** Program entry point.
     * @param args program arguments (unused here)
     */
    public static void main(String[] args) {
        try {

            // configure Orekit
            final File orekitData = getResourceFile("/orekit-data");
            DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(orekitData));

            // read input file
            final File input = getResourceFile("/reference-use-cases/longitude-maneuver.in");
            final Map<ParameterKey, String> parameters = parseInput(input);

            // build the simulator
            LatestEscapeTimeStationKeeping stationKeeping =
                    new LatestEscapeTimeStationKeeping(parameters);

            // perform simulation
            final File output = new File(input.getParentFile(), "longitude-maneuver.dat");
            stationKeeping.run(output);

        } catch (ParseException pe) {
            System.err.println(pe.getLocalizedMessage());
        } catch (IOException ioe) {
            System.err.println(ioe.getLocalizedMessage());
        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
        } catch (SkatException se) {
            System.err.println(se.getLocalizedMessage());
        }
    }

    /** Get a resource file.
     * @param name resource file name
     * @return resource file
     * @exception SkatException if resource cannot be found
     */
    private static File getResourceFile(final String name)
        throws SkatException {
        try {
            URL resourceURL = LatestEscapeTimeStationKeeping.class.getResource(name);
            if (resourceURL == null) {
                throw new SkatException(SkatMessages.UNABLE_TO_FIND_RESOURCE, name);
            }
            return new File(resourceURL.toURI().getPath());        
        } catch (URISyntaxException use) {
            throw new SkatException(use, null);
        }
    }

    /** Simple constructor.
     * @param parameters key/value parameters map
     * @exception SkatException if some data cannot be set up
     * @exception OrekitException if orekit cannot be initialized properly (gravity field, UTC ...)
     * @exception ParseException if gravity field cannot be read
     * @exception IOException if gravity field cannot be read
     */
    public LatestEscapeTimeStationKeeping(final Map<ParameterKey, String> parameters)
            throws SkatException, OrekitException, ParseException, IOException {

        // set up frames
        inertialFrame = getInertialFrame(ParameterKey.INERTIAL_FRAME, parameters);
        final Frame earthFrame = getEarthFrame(ParameterKey.EARTH_FRAME, parameters);

        // set up Earth model
        earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                     Constants.WGS84_EARTH_FLATTENING,
                                     earthFrame);

        // load gravity field
        final PotentialCoefficientsProvider gravityField = GravityFieldFactory.getPotentialProvider();
        final int degree = getInt(ParameterKey.GRAVITY_FIELD_DEGREE, parameters);
        final int order  = getInt(ParameterKey.GRAVITY_FIELD_ORDER,  parameters);
        ForceModel gravity = new CunninghamAttractionModel(earthFrame,
                                                           gravityField.getAe(),
                                                           gravityField.getMu(),
                                                           gravityField.getC(degree, order, false),
                                                           gravityField.getS(degree, order, false));
        // set up orbit
        final Orbit initialOrbit;
        if (parameters.containsKey(ParameterKey.ORBIT_CIRCULAR_DATE)) {
            initialOrbit = new CircularOrbit(getDouble(ParameterKey.ORBIT_CIRCULAR_A,    parameters),
                                                     getDouble(ParameterKey.ORBIT_CIRCULAR_EX,   parameters),
                                                     getDouble(ParameterKey.ORBIT_CIRCULAR_EY,   parameters),
                                                     getAngle(ParameterKey.ORBIT_CIRCULAR_I,     parameters),
                                                     getAngle(ParameterKey.ORBIT_CIRCULAR_RAAN,  parameters),
                                                     getAngle(ParameterKey.ORBIT_CIRCULAR_ALPHA, parameters),
                                                     PositionAngle.MEAN,
                                                     inertialFrame,
                                                     getDate(ParameterKey.ORBIT_CIRCULAR_DATE, parameters),
                                                     gravityField.getMu());
        } else {
            initialOrbit = new EquinoctialOrbit(getDouble(ParameterKey.ORBIT_EQUINOCTIAL_A,     parameters),
                                                getDouble(ParameterKey.ORBIT_EQUINOCTIAL_EX,    parameters),
                                                getDouble(ParameterKey.ORBIT_EQUINOCTIAL_EY,    parameters),
                                                getDouble(ParameterKey.ORBIT_EQUINOCTIAL_HX,    parameters),
                                                getDouble(ParameterKey.ORBIT_EQUINOCTIAL_HY,    parameters),
                                                getAngle(ParameterKey.ORBIT_EQUINOCTIAL_LAMBDA, parameters),
                                                PositionAngle.MEAN,
                                                inertialFrame,
                                                getDate(ParameterKey.ORBIT_EQUINOCTIAL_DATE, parameters),
                                                gravityField.getMu());            
        }
        initialState = new SpacecraftState(initialOrbit);

        // set up propagator, using a 10m propagation tolerance in position
        final double minStep = 0.01;
        final double maxStep = Constants.JULIAN_DAY;
        final double dP      = 10.0;
        final double[][] tolerance = NumericalPropagator.tolerances(dP, initialOrbit, initialOrbit.getType());
        final AdaptiveStepsizeIntegrator integrator = new DormandPrince853Integrator(minStep, maxStep, tolerance[0], tolerance[1]);
        integrator.setInitialStepSize(initialOrbit.getKeplerianPeriod() / 100.0);
        final NumericalPropagator numPropagator = new NumericalPropagator(integrator);
        numPropagator.setAttitudeProvider(new LofOffset(initialState.getFrame(), LOFType.TNW));
        numPropagator.addForceModel(gravity);
        numPropagator.setOrbitType(initialOrbit.getType());
        propagator = numPropagator;

        startDate     = getDate(ParameterKey.SIMULATION_START_DATE, parameters);
        cycleDuration = getDouble(ParameterKey.SIMULATION_CYCLE_DURATION, parameters) * Constants.JULIAN_DAY;
        cyclesNumber  = getInt(ParameterKey.SIMULATION_CYCLE_NUMBER, parameters);

    }

    public void run(final File output) throws IOException, OrekitException {

        propagator.resetInitialState(initialState);
        SpacecraftState startCycleState = propagator.propagate(startDate);

        BrentOptimizer optimizer = new BrentOptimizer(1.0e-6, 1.0e-5);
        EscapeTime escapeTime = new EscapeTime(propagator, earth);

        PrintHandler stepHandler = new PrintHandler(output, earth);
        for (int i = 0; i < cyclesNumber; ++i) {
            escapeTime.setInitialState(startCycleState);
            escapeTime.setTargetDate(initialState.getDate().shiftedBy(cycleDuration));
            UnivariateRealPointValuePair pair = optimizer.optimize(1000, escapeTime, GoalType.MAXIMIZE, -0.1, 0.1);
            System.out.println("cycle " + i +
                               ", evaluations = " + optimizer.getEvaluations() +
                               ", dV = " + pair.getPoint() +
                               ", duration = " + (pair.getValue() / Constants.JULIAN_DAY));
            startCycleState = escapeTime.oneManeuverCycle(pair.getPoint(),
                                                          startCycleState.getDate().shiftedBy(pair.getValue() - 2.0 * Constants.JULIAN_DAY),
                                                          stepHandler);
        }

        stepHandler.close();

    }

    private static class PrintHandler implements OrekitFixedStepHandler {

        private static final long serialVersionUID = 7991738543062196382L;

        private PrintStream out;
        private BodyShape earth;

        public PrintHandler(File file, BodyShape earth) throws IOException {
            out = new PrintStream(file);
            this.earth = earth;
        }

        public void handleStep(SpacecraftState currentState, boolean isLast)
                throws PropagationException {
            try {

                // get spacecraft longitude
                Vector3D position = currentState.getPVCoordinates(earth.getBodyFrame()).getPosition();
                GeodeticPoint gp = earth.transform(position, earth.getBodyFrame(), currentState.getDate());

                // print it
                out.println(currentState.getDate() + " " + FastMath.toDegrees(gp.getLongitude()));
                if (isLast) {
                    out.println("&");
                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }
        }

        public void close() {
            out.close();
        }

    };

    /** Parse an input file.
     * <p>
     * The input file syntax is a set of key=value lines. Blank lines and lines
     * starting with '#' (after whitespace trimming) are silently ignored. The
     * equal sign may be surrounded by space characters. Keys must correspond to
     * the {@link ParameterKey} enumerate constants, given that matching is not
     * case sensitive and that '_' characters may appear as '.' characters in the
     * file. this means that the lines:
     * <pre>
     *   # this is the semi-major axis
     *   orbit.circular.a   = 7231582
     * </pre>
     * are perfectly right and correspond to key {@link ParameterKey#ORBIT_CIRCULAR_A}.
     * </p>
     * @param input input file
     * @return key/value map
     * @exception IOException if input file cannot be read
     * @exception IllegalArgumentException if a line cannot be read properly
     */
    private static Map<LatestEscapeTimeStationKeeping.ParameterKey, String> parseInput(final File input)
            throws IOException, IllegalArgumentException {

        final Map<LatestEscapeTimeStationKeeping.ParameterKey, String> map =
                new HashMap<LatestEscapeTimeStationKeeping.ParameterKey, String>();

        BufferedReader reader = new BufferedReader(new FileReader(input));
        for (String line = reader.readLine(); line != null; line = reader.readLine()) {
            line = line.trim();
            // we ignore blank lines and line starting with '#'
            if ((line.length() > 0) && !line.startsWith("#")) {
                String[] fields = line.split("\\s*=\\s*");
                if (fields.length != 2) {
                    throw new IllegalArgumentException(line);
                }
                ParameterKey key = ParameterKey.valueOf(fields[0].toUpperCase().replaceAll("\\.", "_"));
                map.put(key, fields[1]);
            }
        }
        reader.close();

        return map;

    }

    /** Get a raw string value from a parameters map.
     * @param key parameter key
     * @param parameters key/value map containing the parameters
     * @return string value corresponding to the key
     * @exception NoSuchElementException if key is not in the map
     */
    private static String getString(final ParameterKey key, final Map<ParameterKey, String> parameters)
            throws NoSuchElementException {
        final String value = parameters.get(key);
        if (value == null) {
            throw new NoSuchElementException(key.toString());
        }
        return value.trim();
    }

    /** Get a raw double value from a parameters map.
     * @param key parameter key
     * @param parameters key/value map containing the parameters
     * @return double value corresponding to the key
     * @exception NoSuchElementException if key is not in the map
     */
    private static double getDouble(final ParameterKey key, final Map<ParameterKey, String> parameters)
            throws NoSuchElementException {
        return Double.parseDouble(getString(key, parameters));
    }

    /** Get a raw int value from a parameters map.
     * @param key parameter key
     * @param parameters key/value map containing the parameters
     * @return int value corresponding to the key
     * @exception NoSuchElementException if key is not in the map
     */
    private static int getInt(final ParameterKey key, final Map<ParameterKey, String> parameters)
            throws NoSuchElementException {
        return Integer.parseInt(getString(key, parameters));
    }

    /** Get an angle value from a parameters map.
     * <p>
     * The angle is considered to be in degrees in the file, it will be returned in radians
     * </p>
     * @param key parameter key
     * @param parameters key/value map containing the parameters
     * @return angular value corresponding to the key, in radians
     * @exception NoSuchElementException if key is not in the map
     */
    private static double getAngle(final ParameterKey key, final Map<ParameterKey, String> parameters)
            throws NoSuchElementException {
        return FastMath.toRadians(getDouble(key, parameters));
    }

    /** Get a date value from a parameters map.
     * <p>
     * The date is considered to be in UTC in the file
     * </p>
     * @param key parameter key
     * @param parameters key/value map containing the parameters
     * @return date value corresponding to the key
     * @exception NoSuchElementException if key is not in the map
     * @exception SkatException if UTC time scale cannot be retrieved
     */
    private static AbsoluteDate getDate(final ParameterKey key, final Map<ParameterKey, String> parameters)
            throws NoSuchElementException, OrekitException {
        return new AbsoluteDate(getString(key, parameters), TimeScalesFactory.getUTC());
    }

    /** Get an inertial frame from a parameters map.
     * @param key parameter key
     * @param parameters key/value map containing the parameters
     * @return inertial frame corresponding to the key
     * @exception NoSuchElementException if key is not in the map
     * @exception SkatException if frame is not pseudo-inertial
     * @exception OrekitException if frame cannot be built
     */
    private static Frame getInertialFrame(final ParameterKey key, final Map<ParameterKey, String> parameters)
            throws NoSuchElementException, SkatException, OrekitException {

        // get the name of the desired frame
        final String frameName = getString(key, parameters);

        // check the name against predefined frames
        for (Predefined predefined : Predefined.values()) {
            if (frameName.equals(predefined.getName())) {
                if (FramesFactory.getFrame(predefined).isPseudoInertial()) {
                    return FramesFactory.getFrame(predefined);
                } else {
                    throw new SkatException(SkatMessages.NOT_INERTIAL_FRAME, frameName);
                }
            }
        }

        // none of the frames match the name
        throw new SkatException(SkatMessages.UNKNOWN_FRAME, frameName);

    }

    /** Get an Earth frame from a parameters map.
     * <p>
     * We consider Earth frames are the frames with name starting with "ITRF".
     * </p>
     * @param key parameter key
     * @param parameters key/value map containing the parameters
     * @return Earth frame corresponding to the key
     * @exception NoSuchElementException if key is not in the map
     * @exception SkatException if frame is not pseudo-inertial
     * @exception OrekitException if frame cannot be built
     */
    private static Frame getEarthFrame(final ParameterKey key, final Map<ParameterKey, String> parameters)
            throws NoSuchElementException, SkatException, OrekitException {

        // get the name of the desired frame
        final String frameName = getString(key, parameters);

        // check the name against predefined frames
        for (Predefined predefined : Predefined.values()) {
            if (frameName.equals(predefined.getName())) {
                if (frameName.startsWith("ITRF")) {
                    return FramesFactory.getFrame(predefined);
                } else {
                    throw new SkatException(SkatMessages.NOT_EARTH_FRAME, frameName);
                }
            }
        }

        // none of the frames match the name
        throw new SkatException(SkatMessages.UNKNOWN_FRAME, frameName);

    }

    /** Input parameter keys. */
    private static enum ParameterKey {

        INERTIAL_FRAME,
        EARTH_FRAME,
        GRAVITY_FIELD_DEGREE,
        GRAVITY_FIELD_ORDER,

        ORBIT_CIRCULAR_DATE,
        ORBIT_CIRCULAR_A,
        ORBIT_CIRCULAR_EX,
        ORBIT_CIRCULAR_EY,
        ORBIT_CIRCULAR_I,
        ORBIT_CIRCULAR_RAAN,
        ORBIT_CIRCULAR_ALPHA,

        ORBIT_EQUINOCTIAL_DATE,
        ORBIT_EQUINOCTIAL_A,
        ORBIT_EQUINOCTIAL_EX,
        ORBIT_EQUINOCTIAL_EY,
        ORBIT_EQUINOCTIAL_HX,
        ORBIT_EQUINOCTIAL_HY,
        ORBIT_EQUINOCTIAL_LAMBDA,

        SIMULATION_START_DATE,
        SIMULATION_CYCLE_DURATION,
        SIMULATION_CYCLE_NUMBER;

    }

}
