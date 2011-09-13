/* Copyright 2011 Eumetsat */

package eu.eumetsat.skat;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.net.URISyntaxException;
import java.net.URL;
import java.text.ParseException;

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
import org.orekit.frames.LOFType;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.propagation.sampling.OrekitFixedStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;

import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;
import eu.eumetsat.skat.utils.KeyValueFileParser;
import eu.eumetsat.skat.utils.ParameterKey;

/** Station-Keeping Analysis Tool (SKAT).
 * <p>
 * This tool is a station-keeping simulator prototype for both LEO and GEO orbits.
 * It's aim is to perform very long term simulation, up to the full lifetime
 * of a spacecraft.
 * </p>
 */
public class Skat {

    /** Output file. */
    private final PrintStream output;

    /** Initial orbit. */
    private final SpacecraftState initialState;

    /** Earth model. */
    private final BodyShape earth;

    /** Orbit propagator. */
    private final Propagator propagator;

    /** Inertial frame. */
    private final Frame inertialFrame;

    /** Earth frame. */
    private final Frame earthFrame;

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
            if (args.length != 1) {
                System.err.println("usage: java eu.eumetsat.skat.Skat input-file");
                System.exit(1);
            }

            // build the simulator
            Skat stationKeeping = new Skat(new File(args[0]));

            // perform simulation
            stationKeeping.run();

            // close the output stream
            stationKeeping.close();

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
     * @param input main input file
     * @exception SkatException if some data cannot be set up
     * @exception OrekitException if orekit cannot be initialized properly (gravity field, UTC ...)
     * @exception ParseException if gravity field cannot be read
     * @exception IOException if gravity field cannot be read
     */
    public Skat(final File input)
            throws SkatException, OrekitException, ParseException, IOException {

        // parse input file
        final KeyValueFileParser<ParameterKey> parser =
            new KeyValueFileParser<ParameterKey>(ParameterKey.class);
        parser.parseInput(new FileInputStream(input));
        TimeScale utc = TimeScalesFactory.getUTC();

        // set up frames
        inertialFrame = parser.getInertialFrame(ParameterKey.INERTIAL_FRAME);
        earthFrame    = parser.getEarthFrame(ParameterKey.EARTH_FRAME);

        // set up Earth model
        earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                     Constants.WGS84_EARTH_FLATTENING,
                                     earthFrame);

        // load gravity field
        final PotentialCoefficientsProvider gravityField = GravityFieldFactory.getPotentialProvider();
        final int degree = parser.getInt(ParameterKey.GRAVITY_FIELD_DEGREE);
        final int order  = parser.getInt(ParameterKey.GRAVITY_FIELD_ORDER);
        ForceModel gravity = new CunninghamAttractionModel(earthFrame,
                                                           gravityField.getAe(),
                                                           gravityField.getMu(),
                                                           gravityField.getC(degree, order, false),
                                                           gravityField.getS(degree, order, false));
        // set up orbit
        final Orbit initialOrbit;
        if (parser.containsKey(ParameterKey.ORBIT_CARTESIAN_DATE)) {
            final Vector3D position = parser.getVector(ParameterKey.ORBIT_CARTESIAN_PX,
                                                       ParameterKey.ORBIT_CARTESIAN_PY,
                                                       ParameterKey.ORBIT_CARTESIAN_PZ);
            final Vector3D velocity = parser.getVector(ParameterKey.ORBIT_CARTESIAN_VX,
                                                       ParameterKey.ORBIT_CARTESIAN_VY,
                                                       ParameterKey.ORBIT_CARTESIAN_VZ);
            initialOrbit = new CartesianOrbit(new PVCoordinates(position, velocity),
                                              inertialFrame,
                                              parser.getDate(ParameterKey.ORBIT_CARTESIAN_DATE, utc),
                                              gravityField.getMu());
        } else if (parser.containsKey(ParameterKey.ORBIT_KEPLERIAN_DATE)) {
            initialOrbit = new KeplerianOrbit(parser.getDouble(ParameterKey.ORBIT_KEPLERIAN_A),
                                              parser.getDouble(ParameterKey.ORBIT_KEPLERIAN_E),
                                              parser.getAngle(ParameterKey.ORBIT_KEPLERIAN_I),
                                              parser.getAngle(ParameterKey.ORBIT_KEPLERIAN_PA),
                                              parser.getAngle(ParameterKey.ORBIT_KEPLERIAN_RAAN),
                                              parser.getAngle(ParameterKey.ORBIT_KEPLERIAN_MEAN_ANOMALY),
                                              PositionAngle.MEAN,
                                              inertialFrame,
                                              parser.getDate(ParameterKey.ORBIT_KEPLERIAN_DATE, utc),
                                              gravityField.getMu());
        } else if (parser.containsKey(ParameterKey.ORBIT_CIRCULAR_DATE)) {
            initialOrbit = new CircularOrbit(parser.getDouble(ParameterKey.ORBIT_CIRCULAR_A),
                                             parser.getDouble(ParameterKey.ORBIT_CIRCULAR_EX),
                                             parser.getDouble(ParameterKey.ORBIT_CIRCULAR_EY),
                                             parser.getAngle(ParameterKey.ORBIT_CIRCULAR_I),
                                             parser.getAngle(ParameterKey.ORBIT_CIRCULAR_RAAN),
                                             parser.getAngle(ParameterKey.ORBIT_CIRCULAR_MEAN_LATITUDE_ARGUMENT),
                                             PositionAngle.MEAN,
                                             inertialFrame,
                                             parser.getDate(ParameterKey.ORBIT_CIRCULAR_DATE, utc),
                                             gravityField.getMu());
        } else {
            initialOrbit = new EquinoctialOrbit(parser.getDouble(ParameterKey.ORBIT_EQUINOCTIAL_A),
                                                parser.getDouble(ParameterKey.ORBIT_EQUINOCTIAL_EX),
                                                parser.getDouble(ParameterKey.ORBIT_EQUINOCTIAL_EY),
                                                parser.getDouble(ParameterKey.ORBIT_EQUINOCTIAL_HX),
                                                parser.getDouble(ParameterKey.ORBIT_EQUINOCTIAL_HY),
                                                parser.getAngle(ParameterKey.ORBIT_EQUINOCTIAL_MEAN_LONGITUDE_ARGUMENT),
                                                PositionAngle.MEAN,
                                                inertialFrame,
                                                parser.getDate(ParameterKey.ORBIT_EQUINOCTIAL_DATE, utc),
                                                gravityField.getMu());            
        }
        initialState = new SpacecraftState(initialOrbit);

        // set up propagator
        final double minStep = 0.01;
        final double maxStep = Constants.JULIAN_DAY;
        final double dP      = parser.getDouble(ParameterKey.SIMULATION_POSITION_TOLERANCE);
        final double[][] tolerance = NumericalPropagator.tolerances(dP, initialOrbit, initialOrbit.getType());
        final AdaptiveStepsizeIntegrator integrator = new DormandPrince853Integrator(minStep, maxStep, tolerance[0], tolerance[1]);
        integrator.setInitialStepSize(initialOrbit.getKeplerianPeriod() / 100.0);
        final NumericalPropagator numPropagator = new NumericalPropagator(integrator);
        numPropagator.setAttitudeProvider(new LofOffset(initialState.getFrame(), LOFType.TNW));
        numPropagator.addForceModel(gravity);
        numPropagator.setOrbitType(initialOrbit.getType());
        propagator = numPropagator;

        startDate     = parser.getDate(ParameterKey.SIMULATION_START_DATE, utc);
        cycleDuration = parser.getDouble(ParameterKey.SIMULATION_CYCLE_DURATION) * Constants.JULIAN_DAY;
        cyclesNumber  = parser.getInt(ParameterKey.SIMULATION_CYCLE_NUMBER);

        // open the output stream
        output = new PrintStream(new File(input.getParentFile(),
                                          parser.getString(ParameterKey.OUTPUT_FILE_NAME)));

    }

    /** Run the simulation.
     * @throws OrekitException if some computation cannot be performed
     */
    public void run() throws OrekitException {

        propagator.resetInitialState(initialState);
        SpacecraftState startCycleState = propagator.propagate(startDate);

        BrentOptimizer optimizer = new BrentOptimizer(1.0e-6, 1.0e-5);
        EscapeTime escapeTime = new EscapeTime(propagator, earth);

        PrintHandler stepHandler = new PrintHandler();
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

    }

    /** Close the output stream.
     */
    public void close() {
        output.close();
    }

    private class PrintHandler implements OrekitFixedStepHandler {

        private static final long serialVersionUID = 7991738543062196382L;

        public void handleStep(SpacecraftState currentState, boolean isLast)
                throws PropagationException {
            try {

                // get spacecraft longitude
                Vector3D position = currentState.getPVCoordinates(earth.getBodyFrame()).getPosition();
                GeodeticPoint gp = earth.transform(position, earth.getBodyFrame(), currentState.getDate());

                // print it
                output.println(currentState.getDate() + " " + FastMath.toDegrees(gp.getLongitude()));
                if (isLast) {
                    output.println("&");
                }

            } catch (OrekitException oe) {
                throw new PropagationException(oe);
            }
        }

    };

}
