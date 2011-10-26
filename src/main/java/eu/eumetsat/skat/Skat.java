/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.net.URISyntaxException;
import java.net.URL;
import java.text.ParseException;

import org.antlr.runtime.RecognitionException;
import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.random.RandomGenerator;
import org.apache.commons.math.random.Well19937a;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.frames.Frame;
import org.orekit.orbits.Orbit;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.scenario.Scenario;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.utils.ParameterKey;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatFileParser;
import eu.eumetsat.skat.utils.SkatMessages;
import eu.eumetsat.skat.utils.SupportedScenariocomponent;

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
    private final ScenarioState[] initialStates;

    /** Scenario. */
    private final Scenario scenario;

    /** Earth model. */
    private final BodyShape earth;

    /** Sun model. */
    private final CelestialBody sun;

    /** Inertial frame. */
    private final Frame inertialFrame;

    /** Random generator. */
    private final RandomGenerator generator;

    /** Start date. */
    private final AbsoluteDate startDate;

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

        // general simulation parameters
        final Tree simulationNode = parser.getValue(root, ParameterKey.SIMULATION);
        inertialFrame = parser.getInertialFrame(simulationNode, ParameterKey.SIMULATION_INERTIAL_FRAME);
        earth         = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                             Constants.WGS84_EARTH_FLATTENING,
                                             parser.getEarthFrame(simulationNode, ParameterKey.SIMULATION_EARTH_FRAME));
        sun           = CelestialBodyFactory.getSun();
        startDate     = parser.getDate(simulationNode, ParameterKey.SIMULATION_START_DATE, utc);
        generator     = new Well19937a(parser.getInt(simulationNode, ParameterKey.SIMULATION_RANDOM_SEED));

        // load gravity field
        final double mu = GravityFieldFactory.getPotentialProvider().getMu();

        // set up initial states
        final Tree initialStatesArrayNode = parser.getValue(root, ParameterKey.INITIAL_STATES);
        initialStates = new ScenarioState[parser.getElementsNumber(initialStatesArrayNode)];
        for (int i = 0; i < initialStates.length; ++i) {

            // set up initial state
            final Tree stateNode = parser.getElement(initialStatesArrayNode, i);
            final Tree orbitNode = parser.getValue(stateNode, ParameterKey.INITIAL_STATE_ORBIT);
            final SpacecraftState spacecraftState =
                    new SpacecraftState(parser.getOrbit(orbitNode, inertialFrame, utc, mu),
                                        parser.getDouble(stateNode, ParameterKey.INITIAL_STATE_MASS));
            ScenarioState scenarioState =
                    new ScenarioState(parser.getString(stateNode, ParameterKey.INITIAL_STATE_NAME),
                                      parser.getInt(stateNode, ParameterKey.INITIAL_STATE_CYCLE_NUMBER),
                                      spacecraftState);
            scenarioState = scenarioState.updateInPlaneManeuvers(parser.getInt(stateNode, ParameterKey.INITIAL_STATE_IN_PLANE_MANEUVERS),
                                                                 parser.getDouble(stateNode, ParameterKey.INITIAL_STATE_IN_PLANE_DV));
            scenarioState = scenarioState.updateOutOfPlaneManeuvers(parser.getInt(stateNode, ParameterKey.INITIAL_STATE_OUT_OF_PLANE_MANEUVERS),
                                                                    parser.getDouble(stateNode, ParameterKey.INITIAL_STATE_OUT_OF_PLANE_DV));
            initialStates[i] = scenarioState;
        }

        // set up scenario components
        final Tree scenarioNode = parser.getValue(root, ParameterKey.SCENARIO);
        scenario = new Scenario(parser.getDouble(simulationNode, ParameterKey.SIMULATION_CYCLE_DURATION) * Constants.JULIAN_DAY,
                                parser.getDouble(simulationNode, ParameterKey.SIMULATION_CYCLE_DURATION),
                                earth, sun);
        for (int j = 0; j < parser.getElementsNumber(scenarioNode); ++j) {
            final Tree componentNode = parser.getElement(scenarioNode, j);
            final  String type       = parser.getIdentifier(componentNode, ParameterKey.COMPONENT_TYPE);
            final SupportedScenariocomponent component = SupportedScenariocomponent.valueOf(type);
            scenario.addComponent(component.parse(parser, componentNode, this));
        }

        // TODO add monitorable values

        // open the output stream
        output = new PrintStream(new File(input.getParentFile(),
                                          parser.getString(root, ParameterKey.OUTPUT_FILE_NAME)));

    }

    /** Get the configured random generator.
     * @return configured random generator
     */
    public RandomGenerator getGenerator() {
        return generator;
    }

    /** Get the configured inertial frame.
     * @return configured inertial frame
     */
    public Frame getInertialFrame() {
        return inertialFrame;
    }

    /** Get the configured Earth body.
     * @return configured Earth body
     */
    public BodyShape getEarth() {
        return earth;
    }

    /** Get the configured initial orbit for one spacecraft.
     * @param spacecraftIndex index of the spacecraft considered
     * @return configured initial orbit for the specified spacecraft
     */
    public Orbit getInitialOrbit(final int spacecraftIndex) {
        return initialStates[spacecraftIndex].getRealStartState().getOrbit();
    }

    /** Get the spacecraft index corresponding to a specified name.
     * @param name spacecraft name
     * @return index of the specified spacecraft in the scenario states array
     * @exception SkatException if name is not recognized
     */
    public int getSpacecraftIndex(final String name)
        throws SkatException {

        for (int j = 0; j < initialStates.length; ++j) {
            if (name.equals(initialStates[j].getName())) {
                // we have found an initial state corresponding to the name
                return j;
            }
        }

        // the name was not found in the initial states, generate an error
        final StringBuilder known = new StringBuilder();
        for (final ScenarioState state : initialStates) {
            if (known.length() > 0) {
                known.append(", ");
            }
            known.append(state.getName());
        }
        throw new SkatException(SkatMessages.UNKNOWN_SPACECRAFT, name, known.toString());

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

}
