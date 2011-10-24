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
import org.apache.commons.math.random.RandomGenerator;
import org.apache.commons.math.random.Well19937a;
import org.orekit.bodies.BodyShape;
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

import eu.eumetsat.skat.scenario.ScenarioComponent;
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
    private final List<SpacecraftState> initialState;

    /** Scenario components. */
    private final List<List<ScenarioComponent>> scenarii;

    /** Earth model. */
    private final BodyShape earth;

    /** Inertial frame. */
    private final Frame inertialFrame;

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

        // set up inertial frame
        inertialFrame = parser.getInertialFrame(root, ParameterKey.INERTIAL_FRAME);

        // set up Earth model
        earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                     Constants.WGS84_EARTH_FLATTENING,
                                     parser.getEarthFrame(root, ParameterKey.EARTH_FRAME));

        final Tree simulationNode = parser.getValue(root, ParameterKey.SIMULATION);
        startDate     = parser.getDate(simulationNode, ParameterKey.SIMULATION_START_DATE, utc);
        cycleDuration = parser.getDouble(simulationNode, ParameterKey.SIMULATION_CYCLE_DURATION) * Constants.JULIAN_DAY;
        cyclesNumber  = parser.getInt(simulationNode, ParameterKey.SIMULATION_CYCLE_NUMBER);
        generator     = new Well19937a(parser.getInt(simulationNode, ParameterKey.SIMULATION_RANDOM_SEED));

        // load gravity field
        final double mu = GravityFieldFactory.getPotentialProvider().getMu();

        // set up multi-spacecrafts simulation
        final Tree spacecraftsNode = parser.getValue(root, ParameterKey.SPACECRAFTS);
        initialState = new ArrayList<SpacecraftState>();
        scenarii     = new ArrayList<List<ScenarioComponent>>();
        for (int i = 0; i < parser.getElementsNumber(spacecraftsNode); ++i) {

            // set up orbit
            final Tree orbitNode = parser.getValue(parser.getElement(spacecraftsNode, i),
                                                   ParameterKey.ORBIT);
            final Orbit initialOrbit = parser.getOrbit(orbitNode, inertialFrame, utc, mu);
            initialState.add(new SpacecraftState(initialOrbit));

            // set up scenario components
            final Tree scenarioNode =
                    parser.getValue(parser.getElement(spacecraftsNode, i), ParameterKey.SCENARIO);
            List<ScenarioComponent> components = new ArrayList<ScenarioComponent>();
            for (int j = 0; j < parser.getElementsNumber(scenarioNode); ++j) {
                final Tree componentNode = parser.getElement(scenarioNode, j);
                final  String type = parser.getIdentifier(componentNode, ParameterKey.COMPONENT_TYPE);
                final SupportedScenariocomponent component = SupportedScenariocomponent.valueOf(type);
                components.add(component.parse(parser, componentNode, i, this));
            }
            scenarii.add(components);

        }

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
        return initialState.get(spacecraftIndex).getOrbit();
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
