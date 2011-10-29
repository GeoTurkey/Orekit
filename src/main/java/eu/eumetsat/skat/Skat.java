/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.net.URISyntaxException;
import java.net.URL;
import java.text.ParseException;
import java.util.Calendar;
import java.util.Date;
import java.util.HashSet;
import java.util.Set;
import java.util.TimeZone;

import org.antlr.runtime.RecognitionException;
import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.random.RandomGenerator;
import org.apache.commons.math.random.Well19937a;
import org.apache.commons.math.util.FastMath;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.frames.Frame;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;

import eu.eumetsat.skat.realization.Propagation;
import eu.eumetsat.skat.scenario.Scenario;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.utils.MonitorDuo;
import eu.eumetsat.skat.utils.MonitorMono;
import eu.eumetsat.skat.utils.MonitorableDuo;
import eu.eumetsat.skat.utils.MonitorableMono;
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

    /** Main output file. */
    private final PrintStream output;

    /** Time scale for input/output. */
    private TimeScale utc;

    /** Configured states. */
    private final ScenarioState[] configuredStates;

    /** Propagators managing each spacecraft. */
    final Propagation[] managed;

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

    /** Simulation start date. */
    private final AbsoluteDate startDate;

    /** Cycle duration. */
    private final double cycleDuration;

    /** Number of cycles to use for rolling optimization. */
    private final int rollingCycles;

    /** Mono-spacecraft monitors. */
    private MonitorMono[] monitorsMono;

    /** Duo-spacecrafts monitors. */
    private MonitorDuo[][] monitorsDuo;

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
        utc = TimeScalesFactory.getUTC();

        // general simulation parameters
        final Tree simulationNode = parser.getValue(root, ParameterKey.SIMULATION);
        inertialFrame = parser.getInertialFrame(simulationNode, ParameterKey.SIMULATION_INERTIAL_FRAME);
        earth         = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                             Constants.WGS84_EARTH_FLATTENING,
                                             parser.getEarthFrame(simulationNode, ParameterKey.SIMULATION_EARTH_FRAME));
        sun           = CelestialBodyFactory.getSun();
        startDate     = parser.getDate(simulationNode, ParameterKey.SIMULATION_START_DATE, utc);
        generator     = new Well19937a(parser.getInt(simulationNode, ParameterKey.SIMULATION_RANDOM_SEED));
        cycleDuration = parser.getDouble(simulationNode, ParameterKey.SIMULATION_CYCLE_DURATION);
        rollingCycles = parser.getInt(simulationNode, ParameterKey.SIMULATION_ROLLING_CYCLES);

        // load gravity field
        final double mu = GravityFieldFactory.getPotentialProvider().getMu();

        // set up configured states
        final Tree initialStatesArrayNode = parser.getValue(root, ParameterKey.INITIAL_STATES);
        configuredStates = new ScenarioState[parser.getElementsNumber(initialStatesArrayNode)];
        managed          = new Propagation[configuredStates.length];
        for (int i = 0; i < configuredStates.length; ++i) {

            // set up configured state
            final Tree stateNode = parser.getElement(initialStatesArrayNode, i);
            final Tree orbitNode = parser.getValue(stateNode, ParameterKey.INITIAL_STATE_ORBIT);
            final SpacecraftState spacecraftState =
                    new SpacecraftState(parser.getOrbit(orbitNode, inertialFrame, utc, mu),
                                        parser.getDouble(stateNode, ParameterKey.INITIAL_STATE_MASS));
            ScenarioState scenarioState =
                    new ScenarioState(parser.getString(stateNode, ParameterKey.INITIAL_STATE_NAME),
                                      parser.getDouble(stateNode, ParameterKey.INITIAL_STATE_BOL_MASS),
                                      parser.getInt(stateNode,    ParameterKey.INITIAL_STATE_CYCLE_NUMBER),
                                      spacecraftState);
            scenarioState = scenarioState.updateInPlaneManeuvers(parser.getInt(stateNode, ParameterKey.INITIAL_STATE_IN_PLANE_MANEUVERS),
                                                                 0.0,
                                                                 parser.getDouble(stateNode, ParameterKey.INITIAL_STATE_IN_PLANE_TOTAL_DV));
            scenarioState = scenarioState.updateOutOfPlaneManeuvers(parser.getInt(stateNode, ParameterKey.INITIAL_STATE_OUT_OF_PLANE_MANEUVERS),
                                                                    0.0,
                                                                    parser.getDouble(stateNode, ParameterKey.INITIAL_STATE_OUT_OF_PLANE_TOTAL_DV));
            configuredStates[i] = scenarioState;
        }

        monitorsMono = new MonitorMono[configuredStates.length];
        monitorsDuo  = new MonitorDuo[configuredStates.length][configuredStates.length];
        // TODO create the monitors

        // set up scenario components
        final Tree scenarioNode = parser.getValue(root, ParameterKey.SCENARIO);
        scenario = new Scenario(parser.getDouble(simulationNode, ParameterKey.SIMULATION_CYCLE_DURATION) * Constants.JULIAN_DAY,
                                parser.getDouble(simulationNode, ParameterKey.SIMULATION_CYCLE_DURATION),
                                earth, sun);
        scenario.setCycleEnd(parser.getDate(simulationNode, ParameterKey.SIMULATION_END_DATE, utc));
        for (int j = 0; j < parser.getElementsNumber(scenarioNode); ++j) {
            final Tree componentNode = parser.getElement(scenarioNode, j);
            final  String type       = parser.getIdentifier(componentNode, ParameterKey.COMPONENT_TYPE);
            final SupportedScenariocomponent component = SupportedScenariocomponent.valueOf(type);
            scenario.addComponent(component.parse(parser, componentNode, this));
        }

        // check that every spacecraft is managed by at least one propagation component
        for (int i = 0; i < managed.length; ++i) {
            if (managed[i] == null) {
                throw new SkatException(SkatMessages.SPACECRAFT_NOT_MANAGED, getSpacecraftName(i));
            }
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
        return configuredStates[spacecraftIndex].getRealStartState().getOrbit();
    }

    /** Get the cycle duration.
     * @return cycle duration
     */
    public double getCycleDuration() {
        return cycleDuration;
    }

    /** Get the number of cycles to use for rolling optimization.
     * @return number of cycles to use for rolling optimization
     */
    public int getRollingCycles() {
        return rollingCycles;
    }

    /** Get the spacecraft name corresponding to a specified index.
     * @param index of the specified spacecraft in the scenario states array
     * @return name spacecraft name
     * @see #getSpacecraftIndex(String)
     */
    public String getSpacecraftName(final int index) {
        return configuredStates[index].getName();
    }

    /** Get the spacecraft index corresponding to a specified name.
     * @param name spacecraft name
     * @return index of the specified spacecraft in the scenario states array
     * @exception SkatException if name is not recognized
     * @see #getSpacecraftName(int)
     */
    public int getSpacecraftIndex(final String name)
        throws SkatException {

        for (int j = 0; j < configuredStates.length; ++j) {
            if (name.equals(configuredStates[j].getName())) {
                // we have found an initial state corresponding to the name
                return j;
            }
        }

        // the name was not found in the initial states, generate an error
        final StringBuilder known = new StringBuilder();
        for (final ScenarioState state : configuredStates) {
            if (known.length() > 0) {
                known.append(", ");
            }
            known.append(state.getName());
        }
        throw new SkatException(SkatMessages.UNKNOWN_SPACECRAFT, name, known.toString());

    }

    /** Set the monitor for a single spacecraft.
     * @param index spacecraft index
     * @param monitor monitor for the spacecraft
     */
    public void addMonitorable(final int index,
                               final MonitorableMono monitorable) {
        monitorable.register(configuredStates.length, monitorsMono[index]);
    }

    /** Set the monitor for a spacecrafts pair.
     * @param index1 first spacecraft index
     * @param index2 second spacecraft index
     * @param monitor monitor for the spacecrafts pair
     */
    public void addMonitorable(final int index1, final int index2,
                               final MonitorableDuo monitorable) {
        monitorable.register(configuredStates.length, monitorsDuo[index1][index2]);
    }

    /** Check if a spacecraft is managed by a propagator.
     * @param index spacecraft index
     * @return true if spacecraft is managed by a propagator
     * @see #getSpacecraftIndex(String)
     */
    public boolean isManaged(final int index)
        throws SkatException {
        return managed[index] != null;
    }

    /** Set a spacecraft management indicator.
     * @param index spacecraft index
     * @param propagation propagation component managing the spacecraft
     * @see #getSpacecraftIndex(String)
     */
    public void manage(final int index, final Propagation propagation)
        throws SkatException {
        managed[index] = propagation;
    }

    /** Run the simulation.
     * @exception OrekitException if some computation cannot be performed
     * @exception SkatException if some data is missing in the states
     */
    public void run() throws OrekitException, SkatException {

        // propagate all spacecrafts state to simulation start date
        final Set<Propagation> propagationComponents = new HashSet<Propagation>();
        for (final Propagation propagation : managed) {
            propagationComponents.add(propagation);
        }
        ScenarioState[] initialStates = configuredStates;
        for (final Propagation propagation : propagationComponents) {
            propagation.setCycleEnd(startDate);
            initialStates = propagation.updateStates(initialStates);
        }
        for (int i = 0; i < initialStates.length; ++i) {
            initialStates[i] = initialStates[i].updateRealStartState(initialStates[i].getRealEndState());
        }

        final ScenarioState[] finalStates = scenario.updateStates(initialStates);

        dumpOutput(finalStates);

    }

    /** Dump the final states in the output file.
     * @param finalStates states at simulation end
     * @exception OrekitException if orbit cannot be converted
     */
    private void dumpOutput(final ScenarioState[] finalStates)
        throws OrekitException {

        final Date now = Calendar.getInstance(TimeZone.getTimeZone("Etc/UTC")).getTime();
        final int baseIndent = 4;
        final int keysWidth = 28;

        // print the header
        output.println("# file generated on " + new AbsoluteDate(now, utc).toString(utc));
        output.println("#");
        output.println("initial_states   = [");
        for (int i = 0; i < finalStates.length; ++i) {
            final ScenarioState state = finalStates[i];
            output.println("    {");
            output.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_NAME) +
                           "= \"" + state.getName() + "\";");
            output.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_CYCLE_NUMBER) +
                           "= " + state.getCyclesNumber() + ";");
            output.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_IN_PLANE_MANEUVERS) +
                           "= " + state.getInPlaneManeuvers() + ";");
            output.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_IN_PLANE_TOTAL_DV) +
                           "= " + state.getInPlaneTotalDV() + ";");
            output.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_OUT_OF_PLANE_MANEUVERS) +
                           "= " + state.getOutOfPlaneManeuvers() + ";");
            output.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_OUT_OF_PLANE_TOTAL_DV) +
                           "= " + state.getOutOfPlaneTotalDV() + ";");
            output.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_MASS) +
                           "= " + state.getRealStartState().getMass() + ";");

            output.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_ORBIT) +
                           "= {");
            final Orbit orbit = state.getRealStartState().getOrbit(); 
            switch (orbit.getType()) {
            case CARTESIAN :
                final PVCoordinates pv = orbit.getPVCoordinates(inertialFrame);
                final Vector3D p = pv.getPosition();
                final Vector3D v = pv.getVelocity();
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CARTESIAN_DATE) +
                               "= " + orbit.getDate().toString(utc) + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CARTESIAN_POSITION) +
                               "= [" + p.getX() + ", " + p.getY() + ", " + p.getZ() + "];");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CARTESIAN_VELOCITY) +
                               "= [" + v.getX() + ", " + v.getY() + ", " + v.getZ() + "];");
                break;
            case KEPLERIAN :
                final KeplerianOrbit kep = (KeplerianOrbit) orbit;
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_DATE) +
                               "= " + orbit.getDate().toString(utc) + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_A) +
                               "= " + kep.getA() + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_E) +
                               "= " + kep.getE() + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_I) +
                               "= " + FastMath.toDegrees(kep.getI()) + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_PA) +
                               "= " + FastMath.toDegrees(kep.getPerigeeArgument()) + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_RAAN) +
                               "= " + FastMath.toDegrees(kep.getRightAscensionOfAscendingNode()) + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_ANOMALY) +
                               "= " + FastMath.toDegrees(kep.getMeanAnomaly()) + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ANGLE_TYPE) +
                               "= " + PositionAngle.MEAN + ";");
               break;
            case CIRCULAR :
                final CircularOrbit cir = (CircularOrbit) orbit;
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_DATE) +
                               "= " + orbit.getDate().toString(utc) + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_A) +
                               "= " + cir.getA() + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_EX) +
                               "= " + cir.getCircularEx() + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_EY) +
                               "= " + cir.getCircularEy() + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_I) +
                               "= " + FastMath.toDegrees(cir.getI()) + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_RAAN) +
                               "= " + FastMath.toDegrees(cir.getRightAscensionOfAscendingNode()) + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_LATITUDE_ARGUMENT) +
                               "= " + FastMath.toDegrees(cir.getAlphaM()) + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ANGLE_TYPE) +
                               "= " + PositionAngle.MEAN + ";");
                break;
            case EQUINOCTIAL :
                final EquinoctialOrbit equ = (EquinoctialOrbit) orbit;
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_DATE) +
                               "= " + orbit.getDate().toString(utc) + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_A) +
                               "= " + equ.getA() + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_EX) +
                               "= " + equ.getEquinoctialEx() + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_EY) +
                               "= " + equ.getEquinoctialEy() + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_HX) +
                               "= " + equ.getHx() + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_HY) +
                               "= " + equ.getHy() + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_LONGITUDE_ARGUMENT) +
                               "= " + FastMath.toDegrees(equ.getLM()) + ";");
                output.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ANGLE_TYPE) +
                               "= " + PositionAngle.MEAN + ";");
                break;
            default :
                // this should never happen
                throw SkatException.createInternalError(null);
            }
            output.println(formatIndent(baseIndent) + ((i < finalStates.length - 1) ? "}," : "}"));

        }

    }

    /** Format an indentation string.
     * @param indent number of blank characters
     * @param formated string
     */
    private String formatIndent(final int indent) {
        final StringBuilder builder = new StringBuilder();
        for (int i = 0; i < indent; ++i) {
            builder.append(' ');
        }
        return builder.toString();
    }

    /** Format a parameter key string.
     * @param total number of characters desired
     * @param key key to output
     * @param formated string
     */
    private String formatKey(final int total, final ParameterKey key) {
        final StringBuilder builder = new StringBuilder();
        builder.append(key.getKey());
        while (builder.length() < total) {
            builder.append(' ');
        }
        return builder.toString();
    }

    /** Close the output stream.
     */
    public void close() {
        output.close();
    }

}
