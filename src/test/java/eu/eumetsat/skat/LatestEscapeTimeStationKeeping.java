/* Copyright 2011 Eumetsat */

package eu.eumetsat.skat;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.net.URISyntaxException;
import java.net.URL;
import java.text.ParseException;

import org.antlr.runtime.RecognitionException;
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
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.EquinoctialOrbit;
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

import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatFileParser;
import eu.eumetsat.skat.utils.SkatMessages;

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

            // build the simulator
            LatestEscapeTimeStationKeeping stationKeeping =
                    new LatestEscapeTimeStationKeeping(input);

            // perform simulation
            final File output = new File(input.getParentFile(), "longitude-maneuver.dat");
            stationKeeping.run(output);

        } catch (ParseException pe) {
            System.err.println(pe.getLocalizedMessage());
        } catch (IOException ioe) {
            System.err.println(ioe.getLocalizedMessage());
        } catch (NoSuchFieldException nsfe) {
            System.err.println(nsfe.getLocalizedMessage());
        } catch ( RecognitionException re) {
            System.err.println(re.getLocalizedMessage());
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
            URL resourceURL = Skat.class.getResource(name);
            if (resourceURL == null) {
                throw new SkatException(SkatMessages.UNABLE_TO_FIND_RESOURCE, name);
            }
            return new File(resourceURL.toURI().getPath());        
        } catch (URISyntaxException use) {
            throw new SkatException(use, null);
        }
    }

    /** Simple constructor.
     * @param input parameters input
     * @exception SkatException if some data cannot be set up
     * @exception OrekitException if orekit cannot be initialized properly (gravity field, UTC ...)
     * @exception ParseException if gravity field cannot be read
     * @exception IOException if gravity field cannot be read
     * @exception NoSuchFieldException if a required parameter is missing
     * @exception RecognitionException if there is a syntax error in the input file
     */
    public LatestEscapeTimeStationKeeping(File input)
            throws SkatException, OrekitException, ParseException, IOException,
            NoSuchFieldException, RecognitionException {

        final SkatFileParser<ParameterKey> parser =
                new SkatFileParser<ParameterKey>();
            TimeScale utc = TimeScalesFactory.getUTC();

        // set up frames
        inertialFrame = parser.getInertialFrame(ParameterKey.INERTIAL_FRAME);
        final Frame earthFrame = parser.getEarthFrame(ParameterKey.EARTH_FRAME);

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
        if (parser.containsKey(ParameterKey.ORBIT_CIRCULAR_DATE)) {
            initialOrbit = new CircularOrbit(parser.getDouble(ParameterKey.ORBIT_CIRCULAR_A),
                                             parser.getDouble(ParameterKey.ORBIT_CIRCULAR_EX),
                                             parser.getDouble(ParameterKey.ORBIT_CIRCULAR_EY),
                                             parser.getAngle(ParameterKey.ORBIT_CIRCULAR_I),
                                             parser.getAngle(ParameterKey.ORBIT_CIRCULAR_RAAN),
                                             parser.getAngle(ParameterKey.ORBIT_CIRCULAR_ALPHA),
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
                                                parser.getAngle(ParameterKey.ORBIT_EQUINOCTIAL_LAMBDA),
                                                PositionAngle.MEAN,
                                                inertialFrame,
                                                parser.getDate(ParameterKey.ORBIT_EQUINOCTIAL_DATE, utc),
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

        startDate     = parser.getDate(ParameterKey.SIMULATION_START_DATE, utc);
        cycleDuration = parser.getDouble(ParameterKey.SIMULATION_CYCLE_DURATION) * Constants.JULIAN_DAY;
        cyclesNumber  = parser.getInt(ParameterKey.SIMULATION_CYCLE_NUMBER);

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
