/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.net.URISyntaxException;
import java.net.URL;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.List;

import org.antlr.runtime.RecognitionException;
import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.linear.RealMatrix;
import org.apache.commons.math.optimization.MultivariateRealOptimizer;
import org.apache.commons.math.optimization.direct.BOBYQAOptimizer;
import org.apache.commons.math.optimization.direct.CMAESOptimizer;
import org.apache.commons.math.random.RandomGenerator;
import org.apache.commons.math.random.Well19937a;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.PotentialCoefficientsProvider;
import org.orekit.frames.Frame;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.control.ControlLoop;
import eu.eumetsat.skat.control.SKControl;
import eu.eumetsat.skat.realization.ManeuverDateError;
import eu.eumetsat.skat.realization.ManeuverMagnitudeError;
import eu.eumetsat.skat.realization.OrbitDetermination;
import eu.eumetsat.skat.realization.Propagation;
import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.strategies.geo.EccentricityCircle;
import eu.eumetsat.skat.strategies.geo.LongitudeSlotMargins;
import eu.eumetsat.skat.utils.ParameterKey;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatFileParser;
import eu.eumetsat.skat.utils.SkatMessages;

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
    private final List<SpacecraftState> initialState;

    /** Scenario components. */
    private final List<List<ScenarioComponent>> scenarii;

    /** Earth model. */
    private final BodyShape earth;

    /** Inertial frame. */
    private final Frame inertialFrame;

    /** Earth frame. */
    private final Frame earthFrame;

    /** Random generator. */
    private final RandomGenerator generator;

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
     * @exception NoSuchFieldException if a required parameter is missing
     * @exception RecognitionException if there is a syntax error in the input file
     */
    public Skat(final File input)
            throws SkatException, OrekitException, ParseException, IOException,
                   NoSuchFieldException, RecognitionException {

        // parse input file
        final SkatFileParser parser =
                new SkatFileParser(input.getAbsolutePath(), new FileInputStream(input));
        Tree root = parser.getRoot();
        TimeScale utc = TimeScalesFactory.getUTC();

        // set up frames
        inertialFrame = parser.getInertialFrame(root, ParameterKey.INERTIAL_FRAME);
        earthFrame    = parser.getEarthFrame(root, ParameterKey.EARTH_FRAME);

        // set up Earth model
        earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                     Constants.WGS84_EARTH_FLATTENING,
                                     earthFrame);

        final Tree simulationNode = parser.getValue(root, ParameterKey.SIMULATION);
        startDate     = parser.getDate(simulationNode, ParameterKey.SIMULATION_START_DATE, utc);
        cycleDuration = parser.getDouble(simulationNode, ParameterKey.SIMULATION_CYCLE_DURATION) * Constants.JULIAN_DAY;
        cyclesNumber  = parser.getInt(simulationNode, ParameterKey.SIMULATION_CYCLE_NUMBER);
        generator     = new Well19937a(parser.getInt(simulationNode, ParameterKey.SIMULATION_RANDOM_SEED));

        // load gravity field
        final PotentialCoefficientsProvider gravityField = GravityFieldFactory.getPotentialProvider();

        // set up multi-spacecrafts simulation
        final Tree spacecraftsNode = parser.getValue(root, ParameterKey.SPACECRAFTS);
        initialState = new ArrayList<SpacecraftState>();
        scenarii     = new ArrayList<List<ScenarioComponent>>();
        for (int i = 0; i < parser.getElementsNumber(spacecraftsNode); ++i) {

            // set up orbit
            final Tree orbitNode = parser.getValue(parser.getElement(spacecraftsNode, i),
                                                   ParameterKey.ORBIT);
            final Orbit initialOrbit = parser.getOrbit(orbitNode, inertialFrame, utc, gravityField.getMu());
            initialState.add(new SpacecraftState(initialOrbit));

            // set up scenario components
            final Tree scenarioNode =
                    parser.getValue(parser.getElement(spacecraftsNode, i), ParameterKey.SCENARIO);
            List<ScenarioComponent> components = new ArrayList<ScenarioComponent>();
            for (int j = 0; j < parser.getElementsNumber(scenarioNode); ++j) {
                final Tree componentNode = parser.getElement(scenarioNode, j);
                switch (SupportedScenarioComponent.parse(parser, componentNode, ParameterKey.SCENARIO_COMPONENT)) {
                case ORBIT_DETERMINATION :
                    components.add(parseOrbitDeterminationComponent(parser, componentNode, i));
                    break;
                case CONTROL_LOOP :
                    components.add(parseControlLoopComponent(parser, componentNode, i,
                                                             initialOrbit, gravityField));
                    break;
                case MANEUVER_DATE_ERROR :
                    components.add(parseManeuverDateErrorComponent(parser, componentNode, i));
                    break;
                case MANEUVER_MAGNITUDE_ERROR :
                    components.add(parseManeuverMagnitudeErrorComponent(parser, componentNode, i));
                    break;
                case PROPAGATION :
                    components.add(parsePropagationComponent(parser, componentNode,
                                                             initialOrbit, gravityField));
                    break;
                default :
                    throw SkatException.createInternalError(null);
                }
            }
            scenarii.add(components);

        }

        // open the output stream
        output = new PrintStream(new File(input.getParentFile(),
                                          parser.getString(root, ParameterKey.OUTPUT_FILE_NAME)));

    }

    /** Parse an orbit determination component.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param spacecraftIndex spacecraft index
     * @return parsed component
     * @exception SkatException if type of position angle cannot be parsed
     */
    private ScenarioComponent parseOrbitDeterminationComponent(final SkatFileParser parser, final Tree node,
                                                               final int spacecraftIndex)
        throws SkatException {
        final Tree covarianceNode =
                parser.getValue(node, ParameterKey.COMPONENT_ORBIT_DETERMINATION_COVARIANCE);
        final RealMatrix covariance =
                parser.getCovariance(covarianceNode, ParameterKey.COVARIANCE_MATRIX);
        final PositionAngle positionAngle;
        switch (SupportedAngleTypes.parse(parser, covarianceNode, ParameterKey.COVARIANCE_ANGLE_TYPE)) {
        case TRUE_ANGLE :
            positionAngle = PositionAngle.TRUE;
            break;
        case MEAN_ANGLE :
            positionAngle = PositionAngle.MEAN;
            break;
        case ECCENTRIC_ANGLE :
            positionAngle = PositionAngle.ECCENTRIC;
            break;
        default :
            throw SkatException.createInternalError(null);
        }
        final double small =
                parser.getDouble(covarianceNode, ParameterKey.COVARIANCE_SMALL);
        return new OrbitDetermination(spacecraftIndex, covariance, positionAngle, small, generator);
    }

    /** Parse a control loop component.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param spacecraftIndex spacecraft index
     * @param initialOrbit initial orbit
     * @param gravityField gravity field
     * @return parsed component
     * @exception OrekitException if propagator cannot be set up
     * @exception SkatException if control law cannot be recognized
     */
    private ScenarioComponent parseControlLoopComponent(final SkatFileParser parser, final Tree node,
                                                        final int spacecraftIndex, final Orbit initialOrbit,
                                                        final PotentialCoefficientsProvider gravityField)
        throws OrekitException, SkatException {

        // optimizer
        final int maxEval  = parser.getInt(node, ParameterKey.COMPONENT_CONTROL_LOOP_MAX_EVAL);
        final int nbPoints = parser.getInt(node, ParameterKey.COMPONENT_CONTROL_LOOP_NB_POINTS);
        final MultivariateRealOptimizer optimizer;
        switch (SupportedOptimizer.parse(parser, node, ParameterKey.COMPONENT_CONTROL_LOOP_OPTIMIZER)) {
        case CMA_ES :
            optimizer = new CMAESOptimizer(nbPoints);
            break ;
        case BOBYQA :
            optimizer = new BOBYQAOptimizer(nbPoints);
            break;
        default :
            throw SkatException.createInternalError(null);
        }

        final Propagator propagator = parser.getPropagator(parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_PROPAGATOR),
                                                           initialOrbit, earthFrame, gravityField);


        final ControlLoop loop = new ControlLoop(spacecraftIndex, maxEval, optimizer, propagator);

        // control laws
        final Tree controlsNode = parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_CONTROLS);
        for (int i = 0; i < parser.getElementsNumber(controlsNode); ++i) {
            final Tree control = parser.getElement(controlsNode, i);
            final double scale = parser.getDouble(control, ParameterKey.CONTROL_SCALE);
            final String name  = parser.getString(control, ParameterKey.CONTROL_NAME);
            switch (SupportedControlLaw.parse(parser, control, ParameterKey.CONTROL_TYPE)) {
            case LONGITUDE_MARGINS :
                loop.addControl(scale, parseLongitudeMarginControlLaw(parser, control, name));
                break;
            case ECCENTRICITY_CIRCLE :
                loop.addControl(scale, parseEccentricityCircleControlLaw(parser, control, name));
                break;
            default :
                throw SkatException.createInternalError(null);
            }
        }

        // tunable maneuvers
        final Tree maneuversNode = parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_MANEUVERS);
        for (int i = 0; i < parser.getElementsNumber(maneuversNode); ++i) {
            final Tree maneuver     = parser.getElement(maneuversNode, i);
            final boolean inPlane    = parser.getBoolean(maneuver, ParameterKey.MANEUVERS_IN_PLANE);
            final String name        = parser.getString(maneuver,  ParameterKey.MANEUVERS_NAME);
            final Vector3D direction = parser.getVector(maneuver,  ParameterKey.MANEUVERS_DIRECTION).normalize();
            final double isp         = parser.getDouble(maneuver,  ParameterKey.MANEUVERS_ISP);
            final double dvMin       = parser.getDouble(maneuver,  ParameterKey.MANEUVERS_DV_MIN);
            final double dvMax       = parser.getDouble(maneuver,  ParameterKey.MANEUVERS_DV_MAX);
            final double nominal     = parser.getDouble(maneuver,  ParameterKey.MANEUVERS_NOMINAL_DATE);
            final double dtMin       = parser.getDouble(maneuver,  ParameterKey.MANEUVERS_DT_MIN);
            final double dtMax       = parser.getDouble(maneuver,  ParameterKey.MANEUVERS_DT_MAX);
            loop.addTunableManeuver(new TunableManeuver(name, inPlane, direction, isp,
                                                        dvMin, dvMax, nominal, dtMin, dtMax));
        }

        return loop;

    }

    /** Parse a longitude margins control law.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param name name of the control law
     * @return parsed component
     */
    private SKControl parseLongitudeMarginControlLaw(final SkatFileParser parser, final Tree node,
                                                     final String name) {
        final double eastBoundary = parser.getAngle(node, ParameterKey.CONTROL_LONGITUDE_MARGINS_EAST);
        final double westBoundary = parser.getAngle(node, ParameterKey.CONTROL_LONGITUDE_MARGINS_WEST);
        final double target       = parser.getAngle(node, ParameterKey.CONTROL_LONGITUDE_MARGINS_TARGET);
        final double sampling     = parser.getAngle(node, ParameterKey.CONTROL_SAMPLING);
        return new LongitudeSlotMargins(westBoundary, eastBoundary, target, sampling, earth);
    }

    /** Parse an eccentricity circle control law.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param name name of the control law
     * @return parsed component
     */
    private SKControl parseEccentricityCircleControlLaw(final SkatFileParser parser, final Tree node,
                                                     final String name) {
        final double centerX  = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_CENTER_X);
        final double centerY  = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_CENTER_Y);
        final double radius   = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_RADIUS);
        final double sampling = parser.getAngle(node, ParameterKey.CONTROL_SAMPLING);
        return new EccentricityCircle(centerX, centerY, radius, sampling);
    }

    /** Parse a maneuver date error component.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param spacecraftIndex spacecraft index
     * @return parsed component
     */
    private ScenarioComponent parseManeuverDateErrorComponent(final SkatFileParser parser, final Tree node,
                                                              final int spacecraftIndex) {
        final boolean inPlane =
                parser.getBoolean(node, ParameterKey.COMPONENT_MANEUVER_DATE_ERROR_IN_PLANE);
        final boolean outOfPlane =
                parser.getBoolean(node, ParameterKey.COMPONENT_MANEUVER_DATE_ERROR_OUT_OF_PLANE);
        final double standardDeviation =
                parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_DATE_ERROR_STANDARD_DEVIATION);
        return new ManeuverDateError(spacecraftIndex, inPlane, outOfPlane, standardDeviation, generator);
    }

    /** Parse a maneuver magnitude error component.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param spacecraftIndex spacecraft index
     * @return parsed component
     */
    private ScenarioComponent parseManeuverMagnitudeErrorComponent(final SkatFileParser parser, final Tree node,
                                                                   final int spacecraftIndex) {
        final boolean inPlane =
                parser.getBoolean(node, ParameterKey.COMPONENT_MANEUVER_MAGNITUDE_ERROR_IN_PLANE);
        final boolean outOfPlane =
                parser.getBoolean(node, ParameterKey.COMPONENT_MANEUVER_MAGNITUDE_ERROR_OUT_OF_PLANE);
        final double standardDeviation =
                parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_MAGNITUDE_ERROR_STANDARD_DEVIATION);
        return new ManeuverMagnitudeError(spacecraftIndex, inPlane, outOfPlane, standardDeviation, generator);
    }

    /** Parse a propagation component.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param initialOrbit initial orbit
     * @param gravityField gravity field
     * @return parsed component
     * @exception OrekitException if propagator cannot be set up
     * @exception SkatException if propagator cannot be setup
     */
    private ScenarioComponent parsePropagationComponent(final SkatFileParser parser, final Tree node,
                                                        final Orbit initialOrbit,
                                                        final PotentialCoefficientsProvider gravityField)
        throws OrekitException, SkatException {
        return new Propagation(parser.getPropagator(parser.getValue(node, ParameterKey.COMPONENT_PROPAGATION_PROPAGATOR),
                                                    initialOrbit, earthFrame, gravityField));
    }

    /** Run the simulation.
     * @throws OrekitException if some computation cannot be performed
     */
    public void run() throws OrekitException {
        // TODO implement simulation run
        throw SkatException.createInternalError(null);
    }

    /** Close the output stream.
     */
    public void close() {
        output.close();
    }

    /** Enumerate for scenario components. */
    private enum SupportedScenarioComponent {

        /** Constant for orbit determination scenario component. */
        ORBIT_DETERMINATION("orbit determination"),

        /** Constant for control loop scenario component. */
        CONTROL_LOOP("control loop"),

        /** Constant for maneuver_date_error scenario component. */
        MANEUVER_DATE_ERROR("maneuver date error"),

        /** Constant for maneuver magnitude error scenario component. */
        MANEUVER_MAGNITUDE_ERROR("maneuver magnitude error"),

        /** Constant for propagation scenario component. */
        PROPAGATION("propagation");

        /** Type of the component. */
        private final String type;

        /** Simple constructor.
         * @param type type of the component
         */
        private SupportedScenarioComponent(final String type) {
            this.type = type;
        }

        /** Parse scenario component specification.
         * @param parser input file parser
         * @param node data node containing component configuration parameters
         * @param key key for the type
         * @return parsed scenario component specification
         * @exception SkatException if node does not specify a supported component
         */
        public static SupportedScenarioComponent parse(final SkatFileParser parser, final Tree node,
                                                       final ParameterKey key)
            throws SkatException {

            final String s = parser.getString(node, key);
            for (final SupportedScenarioComponent value : values()) {
                if (s.equals(value.type)) {
                    return value;
                }
            }

            // none of the types matched the input file
            final StringBuilder builder = new StringBuilder();
            for (final SupportedScenarioComponent value : values()) {
                if (builder.length() > 0) {
                    builder.append(", ");
                }
                builder.append(value.type);
            }
            throw new SkatException(SkatMessages.UNSUPPORTED_KEY, s, node.getLine(),
                                    parser.getInputName(), builder.toString());

        }
     

    }

    /** Enumerate for control laws. */
    private enum SupportedControlLaw {

        /** Constant for longitude margins control law. */
        LONGITUDE_MARGINS("longitude margins"),

        /** Constant for eccentricity circle_control law. */
        ECCENTRICITY_CIRCLE("eccentricity circle");

        /** Type of control law. */
        private final String type;

        /** Simple constructor.
         * @param tpe type of control law
         */
        private SupportedControlLaw(final String type) {
            this.type = type;
        }

        /** Parse control law specification.
         * @param parser input file parser
         * @param node data node containing control law configuration parameters
         * @param key key for the type
         * @return parsed control law specification
         * @exception SkatException if node does not specify a supported control law
         */
        public static SupportedControlLaw parse(final SkatFileParser parser, final Tree node,
                                                final ParameterKey key)
            throws SkatException {

            final String s = parser.getString(node, key);
            for (final SupportedControlLaw value : values()) {
                if (s.equals(value.type)) {
                    return value;
                }
            }

            // none of the types matched the input file
            final StringBuilder builder = new StringBuilder();
            for (final SupportedControlLaw value : values()) {
                if (builder.length() > 0) {
                    builder.append(", ");
                }
                builder.append(value.type);
            }
            throw new SkatException(SkatMessages.UNSUPPORTED_KEY, s, node.getLine(),
                                    parser.getInputName(), builder.toString());

        }
     

   }

    /** Enumerate for optimizers. */
    private enum SupportedOptimizer {

        /** Constant for CMA-ES. */
        CMA_ES("CMA_ES"),

        /** Constant for Bobyqa. */
        BOBYQA("bobyqa");

        /** Type of angle. */
        private final String type;

        /** Simple constructor.
         * @param type type of optimizer
         */
        private SupportedOptimizer(final String type) {
            this.type = type;
        }

        /** Parse optimizer specification.
         * @param parser input file parser
         * @param node data node containing orbit configuration parameters
         * @param key key for the type
         * @return parsed optimizer specification
         * @exception SkatException if node does not specify a supported optimizer
         */
        public static SupportedOptimizer parse(final SkatFileParser parser, final Tree node,
                                                final ParameterKey key)
            throws SkatException {

            final String s = parser.getString(node, key);
            for (final SupportedOptimizer value : values()) {
                if (s.equals(value.type)) {
                    return value;
                }
            }

            // none of the types matched the input file
            final StringBuilder builder = new StringBuilder();
            for (final SupportedOptimizer value : values()) {
                if (builder.length() > 0) {
                    builder.append(", ");
                }
                builder.append(value.type);
            }
            throw new SkatException(SkatMessages.UNSUPPORTED_KEY, s, node.getLine(),
                                    parser.getInputName(), builder.toString());

        }
     

    }

    /** Enumerate for angle types. */
    private enum SupportedAngleTypes {

        /** Constant for true angle type. */
        TRUE_ANGLE("true angle"),

        /** Constant for eccentric angle type. */
        ECCENTRIC_ANGLE("eccentric angle"),

        /** Constant for mean angle type. */
        MEAN_ANGLE("mean angle");

        /** Type of angle. */
        private final String type;

        /** Simple constructor.
         * @param type type of angle
         */
        private SupportedAngleTypes(final String type) {
            this.type = type;
        }

        /** Parse angle type specification.
         * @param parser input file parser
         * @param node data node containing orbit configuration parameters
         * @param key key for the type
         * @return parsed angle type specification
         * @exception SkatException if node does not specify a supported angle type
         */
        public static SupportedAngleTypes parse(final SkatFileParser parser, final Tree node,
                                                final ParameterKey key)
            throws SkatException {

            final String s = parser.getString(node, key);
            for (final SupportedAngleTypes value : values()) {
                if (s.equals(value.type)) {
                    return value;
                }
            }

            // none of the types matched the input file
            final StringBuilder builder = new StringBuilder();
            for (final SupportedAngleTypes value : values()) {
                if (builder.length() > 0) {
                    builder.append(", ");
                }
                builder.append(value.type);
            }
            throw new SkatException(SkatMessages.UNSUPPORTED_KEY, s, node.getLine(),
                                    parser.getInputName(), builder.toString());

        }
     

    }

}
