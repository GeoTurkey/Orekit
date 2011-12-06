/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.net.URISyntaxException;
import java.net.URL;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.text.Format;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TimeZone;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.antlr.runtime.RecognitionException;
import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.random.RandomGenerator;
import org.apache.commons.math.random.Well19937a;
import org.apache.commons.math.util.FastMath;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.PotentialCoefficientsProvider;
import org.orekit.frames.Frame;
import org.orekit.frames.TopocentricFrame;
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

import eu.eumetsat.skat.control.SKControl;
import eu.eumetsat.skat.realization.Propagation;
import eu.eumetsat.skat.scenario.Scenario;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.CsvFileMonitor;
import eu.eumetsat.skat.utils.MonitorDuo;
import eu.eumetsat.skat.utils.MonitorMono;
import eu.eumetsat.skat.utils.MonitorableDuoSKData;
import eu.eumetsat.skat.utils.MonitorableMonoSKData;
import eu.eumetsat.skat.utils.ParameterKey;
import eu.eumetsat.skat.utils.SimpleMonitorable;
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

    /** Final state output file. */
    private final PrintStream finalOutput;

    /** Maneuvers output file. */
    private final PrintStream maneuversOutput;

    /** Time scale for input/output. */
    private TimeScale utc;

    /** Configured states. */
    private final ScenarioState[] configuredStates;

    /** Propagators managing each spacecraft. */
    final Propagation[] managed;

    /** Scenario. */
    private final Scenario scenario;

    /** Earth model. */
    private final OneAxisEllipsoid earth;

    /** Sun model. */
    private final CelestialBody sun;

    /** Moon model. */
    private final CelestialBody moon;

    /** Gravity field. */
    private final PotentialCoefficientsProvider gravityField;

    /** Reference ground location. */
    private TopocentricFrame groundLocation;

    /** Inertial frame. */
    private final Frame inertialFrame;

    /** Random generator. */
    private final RandomGenerator generator;

    /** Simulation start date. */
    private final AbsoluteDate startDate;

    /** Cycle duration. */
    private final double cycleDuration;

    /** Output step. */
    private final double outputStep;

    /** Number of cycles to use for rolling optimization. */
    private final int rollingCycles;

    /** Mono-spacecraft monitors. */
    private MonitorMono[] monitorsMono;

    /** Mono-spacecraft monitorables. */
    private List<MonitorableMonoSKData> monitorablesMono;

    /** Duo-spacecrafts monitors. */
    private MonitorDuo[][] monitorsDuo;

    /** Duo-spacecrafts monitorables. */
    private List<MonitorableDuoSKData> monitorablesDuo;

    /** Station-keeping control laws monitoring (residuals part). */
    private final Map<SKControl, SimpleMonitorable> controlsResiduals;

    /** Station-keeping control laws monitoring (violations part). */
    private final Map<SKControl, SimpleMonitorable> controlsViolations;

    /** Program entry point.
     * @param args program arguments (unused here)
     */
    public static void main(String[] args) {

        Skat stationKeeping = null;
        try {

            final long startTime = System.currentTimeMillis();

            // check input file
            if (args.length != 1) {
                System.err.println("usage: java -jar skat.jar input-file");
                System.exit(1);
            }

            // configure Orekit
            final File orekitData = getResourceFile("/orekit-data");
            DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(orekitData));

            // build the simulator
            stationKeeping = new Skat(new File(args[0]));

            // perform simulation
            stationKeeping.run();

            final long endTime = System.currentTimeMillis();
            System.out.println("simulation computing time: " + ((endTime - startTime) / 1000) + " seconds");

        } catch (Exception e) {
            e.printStackTrace(System.err); // TODO remove this statement after validation
            System.err.println(e.getLocalizedMessage());
        } finally {

            if (stationKeeping != null) {
                // close the various output streams
                stationKeeping.close();
            }

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

            // try to find the resource alongside with the application jar (useful for production)
            final String className = "/" + Skat.class.getName().replaceAll("\\.", "/") + ".class";
            final Pattern pattern  = Pattern.compile("jar:file:([^!]+)!" + className + "$");
            final Matcher matcher  = pattern.matcher(Skat.class.getResource(className).toURI().toString());
            if (matcher.matches()) {
                File resourceFile = new File(new File(matcher.group(1)).getParentFile(), name);
                if (resourceFile.exists()) {
                    return resourceFile;
                }
            }

            // try to find the resource in the classpath (useful for development in an IDE)
            URL resourceURL = Skat.class.getResource(name);
            if (resourceURL != null) {
                return new File(resourceURL.toURI().getPath());
            }

            // not found
            throw new SkatException(SkatMessages.UNABLE_TO_FIND_RESOURCE, name);

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
        final Date now = Calendar.getInstance(TimeZone.getTimeZone("Etc/UTC")).getTime();
        final SkatFileParser parser =
                new SkatFileParser(input.getAbsolutePath(), new FileInputStream(input));
        Tree root = parser.getRoot();
        utc = TimeScalesFactory.getUTC();

        // general simulation parameters
        final Tree simulationNode = parser.getValue(root, ParameterKey.SIMULATION);
        final String baseName = parser.getString(simulationNode, ParameterKey.SIMULATION_OUTPUT_BASE_NAME);

        // open the output streams and print the headers
        finalOutput     = new PrintStream(new File(input.getParentFile(), baseName + ".out"));
        finalOutput.println("# file generated on " + new AbsoluteDate(now, utc).toString(utc));
        finalOutput.println("#");
        maneuversOutput = new PrintStream(new File(input.getParentFile(), baseName + ".man"));
        maneuversOutput.println("# file generated on " + new AbsoluteDate(now, utc).toString(utc));
        maneuversOutput.println("#");

        // get general data
        inertialFrame = parser.getInertialFrame(simulationNode, ParameterKey.SIMULATION_INERTIAL_FRAME);
        earth         = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                             Constants.WGS84_EARTH_FLATTENING,
                                             parser.getEarthFrame(simulationNode, ParameterKey.SIMULATION_EARTH_FRAME));
        earth.setAngularThreshold(1.e-7);
        sun           = CelestialBodyFactory.getSun();
        moon          = CelestialBodyFactory.getMoon();
        gravityField  = GravityFieldFactory.getPotentialProvider();
        startDate     = parser.getDate(simulationNode, ParameterKey.SIMULATION_START_DATE, utc);
        generator     = new Well19937a(parser.getInt(simulationNode, ParameterKey.SIMULATION_RANDOM_SEED));
        cycleDuration = parser.getDouble(simulationNode, ParameterKey.SIMULATION_CYCLE_DURATION);
        outputStep    = parser.getDouble(simulationNode, ParameterKey.SIMULATION_OUTPUT_STEP);
        rollingCycles = parser.getInt(simulationNode, ParameterKey.SIMULATION_ROLLING_CYCLES);

        final Tree locationNode = parser.getValue(simulationNode, ParameterKey.SIMULATION_GROUND_LOCATION);
        groundLocation = new TopocentricFrame(earth,
                                              new GeodeticPoint(parser.getAngle(locationNode,  ParameterKey.SIMULATION_GROUND_LOCATION_LATITUDE),
                                                                parser.getAngle(locationNode,  ParameterKey.SIMULATION_GROUND_LOCATION_LONGITUDE),
                                                                parser.getDouble(locationNode, ParameterKey.SIMULATION_GROUND_LOCATION_ALTITUDE)),
                                              "skat-ground-location");

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

            final double initialMass = parser.getDouble(stateNode, ParameterKey.INITIAL_STATE_MASS);
            final double bolMass = parser.getDouble(stateNode, ParameterKey.INITIAL_STATE_BOL_MASS);
            if (initialMass > bolMass) {
                throw new SkatException(SkatMessages.INITIAL_MASS_LARGER_THAN_BOL_MASS,
                                        initialMass, bolMass);
            }

            final SpacecraftState spacecraftState =
                    new SpacecraftState(parser.getOrbit(orbitNode, inertialFrame, utc, mu), initialMass);
            ScenarioState scenarioState =
                    new ScenarioState(parser.getString(stateNode, ParameterKey.INITIAL_STATE_NAME),
                                      inertialFrame, earth, bolMass,
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

        final String headerMarker  = "#";
        final String separator     = ",";
        final Format format        = new DecimalFormat("#0.000000000000000E00", new DecimalFormatSymbols(Locale.US));
        final double dateTolerance = 0.001;

        // create monitors for single spacecraft
        monitorsMono = new MonitorMono[configuredStates.length];
        monitorablesMono = new ArrayList<MonitorableMonoSKData>();
        for (int i = 0; i < configuredStates.length; ++i) {
            final File monoFile = new File(input.getParentFile(),
                                           baseName + "-" + configuredStates[i].getName() + ".csv");
            monitorsMono[i] = new CsvFileMonitor(i, new FileOutputStream(monoFile),
                                                 headerMarker, separator, format, startDate, dateTolerance);
        }

        // create monitors for spacecrafts pair
        monitorsDuo  = new MonitorDuo[configuredStates.length][configuredStates.length];
        monitorablesDuo = new ArrayList<MonitorableDuoSKData>();
        for (int i = 0; i < configuredStates.length - 1; ++i) {
            for (int j = i + 1; j < configuredStates.length; ++j) {
                final File duoFile = new File(input.getParentFile(),
                                              baseName + "-" + configuredStates[i].getName() +
                                              "-" + configuredStates[j].getName() + ".csv");
                monitorsDuo[i][j] = new CsvFileMonitor(i, j, new FileOutputStream(duoFile),
                                                       headerMarker, separator, format, startDate, dateTolerance);
            }
        }

        // add regular data monitoring
        if (parser.containsKey(root, ParameterKey.MONITORING_MONO)) {
            final Tree monitoringNode = parser.getValue(root, ParameterKey.MONITORING_MONO);
            for (int i = 0; i < parser.getElementsNumber(monitoringNode); ++i) {
                MonitorableMonoSKData monitorable =
                        (MonitorableMonoSKData) parser.getEnumerate(monitoringNode, i, MonitorableMonoSKData.class);
                for (int j = 0; j < configuredStates.length; ++j) {
                    monitorable.register(configuredStates.length, monitorsMono[j]);
                }
                monitorablesMono.add(monitorable);
            }
        }

        if (parser.containsKey(root, ParameterKey.MONITORING_DUO)) {
            final Tree monitoringNode = parser.getValue(root, ParameterKey.MONITORING_DUO);
            for (int i = 0; i < parser.getElementsNumber(monitoringNode); ++i) {
                MonitorableDuoSKData monitorable =
                        (MonitorableDuoSKData) parser.getEnumerate(monitoringNode, i, MonitorableDuoSKData.class);
                for (int j = 0; j < configuredStates.length - 1; ++j) {
                    for (int k = j + 1; k < configuredStates.length; ++k) {
                        monitorable.register(configuredStates.length, monitorsDuo[j][k]);
                    }
                }
                monitorablesDuo.add(monitorable);
            }
        }

        controlsResiduals  = new HashMap<SKControl, SimpleMonitorable>();
        controlsViolations = new HashMap<SKControl, SimpleMonitorable>();

        // set up scenario components
        final Tree scenarioNode = parser.getValue(root, ParameterKey.SCENARIO);
        scenario = (Scenario) SupportedScenariocomponent.SCENARIO.parse(parser, scenarioNode, this);
        scenario.setCycleEnd(parser.getDate(simulationNode, ParameterKey.SIMULATION_END_DATE, utc));

        // check that every spacecraft is managed by at least one propagation component
        for (int i = 0; i < managed.length; ++i) {
            if (managed[i] == null) {
                throw new SkatException(SkatMessages.SPACECRAFT_NOT_MANAGED, getSpacecraftName(i));
            }
        }

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
    public OneAxisEllipsoid getEarth() {
        return earth;
    }

    /** Get the configured Sun body.
     * @return configured Sun body
     */
    public CelestialBody getSun() {
        return sun;
    }

    /** Get the configured Moon body.
     * @return configured Moon body
     */
    public CelestialBody getMoon() {
        return moon;
    }

    /** Get the configured gravity field.
     * @return configured gravity field
     */
    public PotentialCoefficientsProvider getgravityField() {
        return gravityField;
    }

    /** Get the configured ground location.
     * @return configured ground location
     */
    public TopocentricFrame getGroundLocation() {
        return groundLocation;
    }

    /** Get the list of mono-spacecraft monitorables.
     * @return list of mono-spacecraft monitorables
     */
    public List<MonitorableMonoSKData> getMonitorablesMono() {
        return monitorablesMono;
    }

    /** Get the list of duo-spacecraft monitorables.
     * @return list of duo-spacecraft monitorables
     */
    public List<MonitorableDuoSKData> getMonitorablesDuo() {
        return monitorablesDuo;
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

    /** Get the output step.
     * @return output step
     */
    public double getOutputStep() {
        return outputStep;
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

    /** Add a control law to be monitored.
     * @param controlLaw control law to add
     * @exception SkatException if index of controlled spacecraft cannot be determined
     */
    public void addControl(final SKControl controlLaw)
        throws SkatException {

        final SimpleMonitorable monitorableResidual =
                new SimpleMonitorable(1, "control law residual: " + controlLaw.getName());
        controlsResiduals.put(controlLaw, monitorableResidual);

        final SimpleMonitorable monitorableViolation;
        if (controlLaw.isConstrained()) {
          monitorableViolation = new SimpleMonitorable(1, "control law violation: " + controlLaw.getName());
          controlsViolations.put(controlLaw, monitorableViolation);
        } else {
            monitorableViolation = null;
        }

        if (controlLaw.getReferenceSpacecraftName() == null) {
            // this is a control law for a single spacecraft
            final int index = getSpacecraftIndex(controlLaw.getControlledSpacecraftName());
            monitorableResidual.register(configuredStates.length, monitorsMono[index]);
            if (monitorableViolation != null) {
                monitorableViolation.register(configuredStates.length, monitorsMono[index]);
            }
        } else {
            // this is a control law for a spacecrafts pair
            final int index1 = getSpacecraftIndex(controlLaw.getControlledSpacecraftName());
            final int index2 = getSpacecraftIndex(controlLaw.getReferenceSpacecraftName());
            monitorableResidual.register(configuredStates.length, monitorsDuo[index1][index2]);
            if (monitorableViolation != null) {
                monitorableViolation.register(configuredStates.length, monitorsDuo[index1][index2]);
            }
        }

    }

    /** Get the control laws residuals monitoring map.
     * @return control laws residuals monitoring map
     */
    public Map<SKControl, SimpleMonitorable> getControlLawsResidualsMap() {
        return controlsResiduals;
    }

    /** Get the control laws violations monitoring map.
     * @return control laws violations monitoring map
     */
    public Map<SKControl, SimpleMonitorable> getControlLawsViolationsMap() {
        return controlsViolations;
    }

    /**
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

    /** Get the maneuvers output stream.
     * @return maneuvers output stream
     */
    public PrintStream getManeuversOutput() {
        return maneuversOutput;
    }

    /** Run the simulation.
     * @exception OrekitException if some computation cannot be performed
     * @exception SkatException if some data is missing in the states
     */
    public void run() throws OrekitException, SkatException {

        final Set<Propagation> propagationComponents = new HashSet<Propagation>();
        for (final Propagation propagation : managed) {
            propagationComponents.add(propagation);
        }

        // set up empty maneuvers lists
        ScenarioState[] initialStates = new ScenarioState[configuredStates.length];
        for (int i = 0; i < initialStates.length; ++i) {
            initialStates[i] = configuredStates[i].updateManeuvers(new ArrayList<ScheduledManeuver>());
        }

        // propagate all spacecrafts state to simulation start date
        for (final Propagation propagation : propagationComponents) {
            propagation.setCycleEnd(startDate);
            initialStates = propagation.updateStates(initialStates);
        }
        for (int i = 0; i < initialStates.length; ++i) {
            initialStates[i] = initialStates[i].updateRealStartState(initialStates[i].getRealEndState());
        }

        // perform the complete simulation
        final ScenarioState[] finalStates = scenario.updateStates(initialStates);

        dumpOutput(finalStates);

    }

    /** Dump the final states in the output file.
     * @param finalStates states at simulation end
     * @exception OrekitException if orbit cannot be converted
     */
    private void dumpOutput(final ScenarioState[] finalStates)
        throws OrekitException {

        final int baseIndent = 4;
        final int keysWidth = 28;

        finalOutput.println("initial_states   = [");
        for (int i = 0; i < finalStates.length; ++i) {
            final ScenarioState state = finalStates[i];
            finalOutput.println("    {");
            finalOutput.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_NAME) +
                           "= \"" + state.getName() + "\";");
            finalOutput.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_CYCLE_NUMBER) +
                           "= " + state.getCyclesNumber() + ";");
            finalOutput.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_IN_PLANE_MANEUVERS) +
                           "= " + state.getInPlaneManeuvers() + ";");
            finalOutput.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_IN_PLANE_TOTAL_DV) +
                           "= " + state.getInPlaneTotalDV() + ";");
            finalOutput.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_OUT_OF_PLANE_MANEUVERS) +
                           "= " + state.getOutOfPlaneManeuvers() + ";");
            finalOutput.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_OUT_OF_PLANE_TOTAL_DV) +
                           "= " + state.getOutOfPlaneTotalDV() + ";");
            finalOutput.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_MASS) +
                           "= " + state.getRealStartState().getMass() + ";");

            finalOutput.println(formatIndent(baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_ORBIT) +
                           "= {");
            final Orbit orbit = state.getRealStartState().getOrbit(); 
            switch (orbit.getType()) {
            case CARTESIAN :
                final PVCoordinates pv = orbit.getPVCoordinates(inertialFrame);
                final Vector3D p = pv.getPosition();
                final Vector3D v = pv.getVelocity();
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CARTESIAN_DATE) +
                               "= " + orbit.getDate().toString(utc) + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CARTESIAN_POSITION) +
                               "= [" + p.getX() + ", " + p.getY() + ", " + p.getZ() + "];");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CARTESIAN_VELOCITY) +
                               "= [" + v.getX() + ", " + v.getY() + ", " + v.getZ() + "];");
                break;
            case KEPLERIAN :
                final KeplerianOrbit kep = (KeplerianOrbit) orbit;
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_DATE) +
                               "= " + orbit.getDate().toString(utc) + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_A) +
                               "= " + kep.getA() + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_E) +
                               "= " + kep.getE() + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_I) +
                               "= " + FastMath.toDegrees(kep.getI()) + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_PA) +
                               "= " + FastMath.toDegrees(kep.getPerigeeArgument()) + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_RAAN) +
                               "= " + FastMath.toDegrees(kep.getRightAscensionOfAscendingNode()) + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_ANOMALY) +
                               "= " + FastMath.toDegrees(kep.getMeanAnomaly()) + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ANGLE_TYPE) +
                               "= " + PositionAngle.MEAN + ";");
               break;
            case CIRCULAR :
                final CircularOrbit cir = (CircularOrbit) orbit;
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_DATE) +
                               "= " + orbit.getDate().toString(utc) + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_A) +
                               "= " + cir.getA() + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_EX) +
                               "= " + cir.getCircularEx() + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_EY) +
                               "= " + cir.getCircularEy() + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_I) +
                               "= " + FastMath.toDegrees(cir.getI()) + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_RAAN) +
                               "= " + FastMath.toDegrees(cir.getRightAscensionOfAscendingNode()) + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_LATITUDE_ARGUMENT) +
                               "= " + FastMath.toDegrees(cir.getAlphaM()) + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ANGLE_TYPE) +
                               "= " + PositionAngle.MEAN + ";");
                break;
            case EQUINOCTIAL :
                final EquinoctialOrbit equ = (EquinoctialOrbit) orbit;
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_DATE) +
                               "= " + orbit.getDate().toString(utc) + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_A) +
                               "= " + equ.getA() + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_EX) +
                               "= " + equ.getEquinoctialEx() + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_EY) +
                               "= " + equ.getEquinoctialEy() + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_HX) +
                               "= " + equ.getHx() + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_HY) +
                               "= " + equ.getHy() + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_LONGITUDE_ARGUMENT) +
                               "= " + FastMath.toDegrees(equ.getLM()) + ";");
                finalOutput.println(formatIndent(2 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ANGLE_TYPE) +
                               "= " + PositionAngle.MEAN + ";");
                break;
            default :
                // this should never happen
                throw SkatException.createInternalError(null);
            }
            finalOutput.println(formatIndent(baseIndent) + ((i < finalStates.length - 1) ? "}," : "}"));

        }
        finalOutput.println("]");

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

    /** Close the various output streams.
     */
    public void close() {
        for (final MonitorMono monitor : monitorsMono) {
            monitor.stopMonitoring();
        }
        for (final MonitorDuo[] row : monitorsDuo) {
            for (final MonitorDuo monitor : row) {
                if (monitor != null) {
                    monitor.stopMonitoring();
                }
            }
        }
        finalOutput.close();
        maneuversOutput.close();
    }

}
