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
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.random.Well19937a;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.MathUtils;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.forces.drag.Atmosphere;
import org.orekit.forces.drag.DTM2000;
import org.orekit.forces.drag.MarshallSolarActivityFutureEstimation;
import org.orekit.forces.drag.MarshallSolarActivityFutureEstimation.StrengthLevel;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
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
import eu.eumetsat.skat.strategies.TunableManeuver;
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
import eu.eumetsat.skat.utils.SkatParser;
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

    /** Atmosphere. */
    private final Atmosphere atmosphere;

    /** Reference ground location. */
    private TopocentricFrame groundLocation;

    /** Inertial frame. */
    private final Frame inertialFrame;

    /** Random generator. */
    private final RandomGenerator generator;

    /** Simulation start date. */
    private final AbsoluteDate startDate;

    /** Simulation end date. */
    private final AbsoluteDate endDate;

    /** Output step. */
    private final double outputStep;

    /** Cycle duration. */
    private double cycleDuration;

    /** Mono-spacecraft monitors. */
    private MonitorMono[] monitorsMono;

    /** Mono-spacecraft monitorables. */
    private List<MonitorableMonoSKData> monitorablesMono;

    /** Duo-spacecrafts monitors. */
    private MonitorDuo[][] monitorsDuo;

    /** Duo-spacecrafts monitorables. */
    private List<MonitorableDuoSKData> monitorablesDuo;

    /** Maneuvers monitorables. */
    private final Map<String, SimpleMonitorable> maneuversMonitorables;

    /** Station-keeping control laws monitoring (margins part). */
    private final Map<SKControl, SimpleMonitorable> controlsValues;

    /** Station-keeping control laws monitoring (violations part). */
    private final Map<SKControl, SimpleMonitorable> controlsViolations;

    /** Pool of maneuvers models. */
    private final TunableManeuver[] maneuversModelsPool;

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
            e.printStackTrace(System.err); // TODO remove once validated
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
        maneuversOutput.println("# column 01: spacecraft name");
        maneuversOutput.println("# column 02: maneuver name");
        maneuversOutput.println("# column 03: maneuver mean date");
        maneuversOutput.println("# column 04: X component of delta V (m/s)");
        maneuversOutput.println("# column 05: Y component of delta V (m/s)");
        maneuversOutput.println("# column 06: Z component of delta V (m/s)");
        maneuversOutput.println("# column 07: delta V (m/s)");
        maneuversOutput.println("# column 08: maneuver duration (s)");
        maneuversOutput.println("# column 09: spacecraft mass before manoeuver (kg)");
        maneuversOutput.println("# column 10: spacecraft mass after manoeuver (kg)");
        maneuversOutput.println("# column 11: model thrust (N)");
        maneuversOutput.println("# column 12: model Isp (s)");
        maneuversOutput.println("# column 13: maneuver thrust (N)");
        maneuversOutput.println("# column 14: maneuver Isp (s)");
        maneuversOutput.println("# column 15: eclipse ratio (if no eclipse constraint, -1)");
        maneuversOutput.println("# column 16: lost eclipse ratio (if no eclipse constraint, -1)");
        maneuversOutput.println("# column 17: yaw angle (rad)");
        maneuversOutput.println("# column 18: replanned flag (if not replanned, empty column)");
        maneuversOutput.println("#");

        // get general data
        cycleDuration = 0.0;
        inertialFrame = parser.getInertialFrame(simulationNode, ParameterKey.SIMULATION_INERTIAL_FRAME);
        earth         = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                             Constants.WGS84_EARTH_FLATTENING,
                                             parser.getEarthFrame(simulationNode, ParameterKey.SIMULATION_EARTH_FRAME));
        earth.setAngularThreshold(1.e-7);
        sun           = CelestialBodyFactory.getSun();
        moon          = CelestialBodyFactory.getMoon();

        // atmospheric model
        final String supportedNames = "(?:Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Oct|Nov|Dec)\\p{Digit}\\p{Digit}\\p{Digit}\\p{Digit}F10\\.(?:txt|TXT)";
        final StrengthLevel strengthLevel = (StrengthLevel) parser.getEnumerate(simulationNode,
                                                                                ParameterKey.SIMULATION_SOLAR_ACTIVITY_STRENGTH,
                                                                                StrengthLevel.class);
        MarshallSolarActivityFutureEstimation msafe = new MarshallSolarActivityFutureEstimation(supportedNames, strengthLevel);
        DataProvidersManager.getInstance().feed(msafe.getSupportedNames(), msafe);
        atmosphere = new DTM2000(msafe, sun, earth);

        startDate     = parser.getDate(simulationNode, ParameterKey.SIMULATION_START_DATE, utc);
        endDate       = parser.getDate(simulationNode, ParameterKey.SIMULATION_END_DATE, utc);
        generator     = new Well19937a(parser.getInt(simulationNode, ParameterKey.SIMULATION_RANDOM_SEED));
        outputStep    = parser.getDouble(simulationNode, ParameterKey.SIMULATION_OUTPUT_STEP);

        final Tree locationNode = parser.getValue(simulationNode, ParameterKey.SIMULATION_GROUND_LOCATION);
        groundLocation = new TopocentricFrame(earth,
                                              new GeodeticPoint(parser.getAngle(locationNode,  ParameterKey.SIMULATION_GROUND_LOCATION_LATITUDE),
                                                                parser.getAngle(locationNode,  ParameterKey.SIMULATION_GROUND_LOCATION_LONGITUDE),
                                                                parser.getDouble(locationNode, ParameterKey.SIMULATION_GROUND_LOCATION_ALTITUDE)),
                                              "skat-ground-location");

        // load gravity field
        final double mu = GravityFieldFactory.getUnnormalizedProvider(2, 2).getMu();

        // maneuvers models pool
        final Tree maneuversNode = parser.getValue(simulationNode, ParameterKey.SIMULATION_MANEUVERS);
        final int nbManeuvers = parser.getElementsNumber(maneuversNode);
        maneuversModelsPool = new TunableManeuver[nbManeuvers];
        for (int i = 0; i < nbManeuvers; ++i) {
            final Tree maneuver = parser.getElement(maneuversNode, i);

            final String name                   = parser.getString(maneuver, ParameterKey.MANEUVERS_NAME);
            final Vector3D direction            = parser.getVector(maneuver, ParameterKey.MANEUVERS_DIRECTION).normalize();
            final double[][] thrust             = parser.getDoubleArray2(maneuver,  ParameterKey.MANEUVERS_THRUST);
            final double[][] isp                = parser.getDoubleArray2(maneuver,  ParameterKey.MANEUVERS_ISP_CURVE);
            final double dvConvergence          = parser.getDouble(maneuver, ParameterKey.MANEUVERS_DV_CONVERGENCE);
            final double dtConvergence          = parser.getDouble(maneuver, ParameterKey.MANEUVERS_DT_CONVERGENCE);
            final double elimination            = parser.getDouble(maneuver, ParameterKey.MANEUVERS_ELIMINATION_THRESHOLD);
            final boolean isPreviousSlew        = parser.containsKey(maneuver,ParameterKey.MANEUVERS_PREVIOUS_SLEW_DELTA_MASS);
            final Vector3D previousSlewDeltaV   = isPreviousSlew ? parser.getVector(maneuver, ParameterKey.MANEUVERS_PREVIOUS_SLEW_DELTA_V) : new Vector3D(0.,0.,0.);
            final double previousSlewDeltaMass  = isPreviousSlew ? parser.getDouble(maneuver, ParameterKey.MANEUVERS_PREVIOUS_SLEW_DELTA_MASS) : 0.0;
            final double previousSlewDelay      = isPreviousSlew ? parser.getDouble(maneuver, ParameterKey.MANEUVERS_PREVIOUS_SLEW_DELAY) : 0.0;
            final boolean isFollowingSlew       = parser.containsKey(maneuver,ParameterKey.MANEUVERS_FOLLOWING_SLEW_DELTA_MASS);
            final Vector3D followingSlewDeltaV  = isFollowingSlew ? parser.getVector(maneuver, ParameterKey.MANEUVERS_FOLLOWING_SLEW_DELTA_V) : new Vector3D(0.,0.,0.);
            final double followingSlewDeltaMass = isFollowingSlew ? parser.getDouble(maneuver, ParameterKey.MANEUVERS_FOLLOWING_SLEW_DELTA_MASS) : 0.0;
            final double followingSlewDelay     = isFollowingSlew ? parser.getDouble(maneuver, ParameterKey.MANEUVERS_FOLLOWING_SLEW_DELAY) : 0.0;
            
            // dv_inf field may either be double[n][2] (variable, function of mass)
            // or double (constant, for backward compatibility) 
            double[][] dvInf;
            if(parser.getValue(maneuver, ParameterKey.MANEUVERS_DV_INF).getType()
            		== SkatParser.ARRAY){
             	dvInf = parser.getDoubleArray2(maneuver, ParameterKey.MANEUVERS_DV_INF);
            }
            else{
             	dvInf = new double[1][2];
            	dvInf[0][0] = parser.getDouble(maneuver, ParameterKey.MANEUVERS_DV_INF);
            	dvInf[0][1] = 0.;
            }
            
            // dv_sup field may either be double[n][2] (variable, function of mass)
            // or double (constant, for backward compatibility) 
            double[][] dvSup;
            if(parser.getValue(maneuver, ParameterKey.MANEUVERS_DV_SUP).getType()
            		== SkatParser.ARRAY){
             	dvSup = parser.getDoubleArray2(maneuver, ParameterKey.MANEUVERS_DV_SUP);
            }
            else{
             	dvSup = new double[1][2];
            	dvSup[0][0] = parser.getDouble(maneuver, ParameterKey.MANEUVERS_DV_SUP);
            	dvSup[0][1] = 0.;
            }
            
            maneuversModelsPool[i] = new TunableManeuver(name, direction, thrust, isp, elimination,
                                                         dvInf, dvSup, dvConvergence, dtConvergence, startDate, endDate,
                                                         isPreviousSlew,  previousSlewDeltaV,  previousSlewDeltaMass,  previousSlewDelay,
                                                         isFollowingSlew, followingSlewDeltaV, followingSlewDeltaMass, followingSlewDelay);
        }

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
            for (TunableManeuver m : maneuversModelsPool) {
                // default setting: all maneuvers stats set to 0
                scenarioState = scenarioState.updateManeuverStats(m.getName(), 0, 0.0, 0.0);
            }
            if (parser.containsKey(stateNode, ParameterKey.INITIAL_STATE_MANEUVERS)) {
                // specific setting: the input file contains initial statistics
                // we overwrite the default values for the provided maneuvers
                final Tree manStatsArrayNode = parser.getValue(stateNode, ParameterKey.INITIAL_STATE_MANEUVERS);
                final int nbProvided = parser.getElementsNumber(maneuversNode);
                for (int j = 0; j < nbProvided; ++j) {
                    final Tree maneuverNode = parser.getElement(manStatsArrayNode, j);
                    scenarioState = scenarioState.updateManeuverStats(parser.getString(maneuverNode,
                                                                                       ParameterKey.INITIAL_STATE_MANEUVER_NAME),
                                                                      parser.getInt(maneuverNode,
                                                                                    ParameterKey.INITIAL_STATE_MANEUVER_NUMBER),
                                                                      0.0,
                                                                      parser.getDouble(maneuverNode,
                                                                                       ParameterKey.INITIAL_STATE_MANEUVER_TOTAL_DV));
                }
            }
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

        // add maneuvers monitoring
        maneuversMonitorables = new HashMap<String, SimpleMonitorable>();
        for (final TunableManeuver maneuver : maneuversModelsPool) {
            SimpleMonitorable monitorable  = new SimpleMonitorable(3, maneuver.getName());
            maneuversMonitorables.put(maneuver.getName(), monitorable);
            for (final MonitorMono monitor : monitorsMono) {
                monitorable.register(configuredStates.length, monitor);
            }
        }

        controlsValues    = new HashMap<SKControl, SimpleMonitorable>();
        controlsViolations = new HashMap<SKControl, SimpleMonitorable>();

        // set up scenario components
        scenario = (Scenario) SupportedScenariocomponent.SCENARIO.parse(parser, root, this);

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

    /** Get the configured atmosphere.
     * @return configured atmosphere
     */
    public Atmosphere getAtmosphere() {
        return atmosphere;
    }

    /** Get the configured ground location.
     * @return configured ground location
     */
    public TopocentricFrame getGroundLocation() {
        return groundLocation;
    }

    /** Get the pool of maneuvers models.
     * @return pool of maneuvers models
     */
    public TunableManeuver[] getManeuversModelsPool() {
        return maneuversModelsPool.clone();
    }

    /** Get a maneuver model.
     * @param name name of the maneuver
     * @return maneuver model
     * @exception SkatException if maneuver cannot be found
     */
    public TunableManeuver getManeuver(final String name)
        throws SkatException {
        for (final TunableManeuver maneuver : maneuversModelsPool) {
            if (maneuver.getName().equals(name)) {
                return maneuver;
            }
        }
        throw new SkatException(SkatMessages.MANEUVER_NOT_FOUND, name);
    }

    /** Get the maneuvers monitoring map.
     * @return maneuvers monitoring map
     */
    public Map<String, SimpleMonitorable> getManeuversMonitorables() {
        return maneuversMonitorables;
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
        return configuredStates[spacecraftIndex].getRealState().getOrbit();
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

        cycleDuration = FastMath.max(cycleDuration, controlLaw.getTimeHorizon());

        final SimpleMonitorable monitorableValue =
                new SimpleMonitorable(1, "control law value: " + controlLaw.getName());
        controlsValues.put(controlLaw, monitorableValue);

        final SimpleMonitorable monitorableViolation;
        if (controlLaw.isConstrained()) {
          monitorableViolation = new SimpleMonitorable(1, "control law violation: " + controlLaw.getName());
          controlsViolations.put(controlLaw, monitorableViolation);
        } else {
            monitorableViolation = null;
        }

        if (controlLaw.getReferenceSpacecraftName() == null) {
            // this is a control law for a single spacecraft
            final int index = controlLaw.getControlledSpacecraftIndex();
            monitorableValue.register(configuredStates.length, monitorsMono[index]);
            if (monitorableViolation != null) {
                monitorableViolation.register(configuredStates.length, monitorsMono[index]);
            }
        } else {
            // this is a control law for a spacecraft pair
            final int index1 = controlLaw.getControlledSpacecraftIndex();
            final int index2 = controlLaw.getReferenceSpacecraftIndex();
            monitorableValue.register(configuredStates.length, monitorsDuo[index1][index2]);
            if (monitorableViolation != null) {
                monitorableViolation.register(configuredStates.length, monitorsDuo[index1][index2]);
            }
        }

    }

    /** Get the control laws values monitoring map.
     * @return control laws values monitoring map
     */
    public Map<SKControl, SimpleMonitorable> getControlLawsValuesMap() {
        return controlsValues;
    }

    /** Get the end date of the simulation.
     * @return end date of the simulation
     */
    public AbsoluteDate getEndDate() {
        return endDate;
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

        // set up state for initial propagation
        ScenarioState[] initialStates = new ScenarioState[configuredStates.length];
        for (int i = 0; i < initialStates.length; ++i) {
            initialStates[i] = configuredStates[i].updateManeuvers(new ArrayList<ScheduledManeuver>());
            initialStates[i] = initialStates[i].updateTargetCycleEnd(startDate);
        }

        // propagate all spacecrafts state to simulation start date
        for (final Propagation propagation : propagationComponents) {
            initialStates = propagation.updateStates(initialStates);
        }

        // perform the complete simulation
        for (int i = 0; i < initialStates.length; ++i) {
            initialStates[i] = initialStates[i].updateTargetCycleEnd(endDate);
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

        final int baseIndent = 4;
        final int keysWidth = 28;

        finalOutput.println("initial_states   = [");
        for (int i = 0; i < finalStates.length; ++i) {
            final ScenarioState state = finalStates[i];
            finalOutput.println("    {");
            finalOutput.println(formatIndent(2 * baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_NAME) +
                           "= \"" + state.getName() + "\";");
            finalOutput.println(formatIndent(2 * baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_CYCLE_NUMBER) +
                           "= " + state.getCyclesNumber() + ";");
            finalOutput.println(formatIndent(2 * baseIndent) +
                                formatKey(keysWidth, ParameterKey.INITIAL_STATE_BOL_MASS) +
                                "= " + state.getBOLMass() + ";");
            finalOutput.println(formatIndent(2 * baseIndent) +
                                formatKey(keysWidth, ParameterKey.INITIAL_STATE_MASS) +
                                "= " + state.getRealState().getMass() + ";");

            finalOutput.println(formatIndent(2 * baseIndent) +
                                formatKey(keysWidth, ParameterKey.INITIAL_STATE_MANEUVERS) +
                                "= [");
            for (int j = 0; j < maneuversModelsPool.length; ++j) {
                final String name = maneuversModelsPool[j].getName();
                finalOutput.println(formatIndent(3 * baseIndent) + "{");
                finalOutput.println(formatIndent(4 * baseIndent) +
                                    formatKey(keysWidth - 2 * baseIndent, ParameterKey.INITIAL_STATE_MANEUVER_NAME) +
                                    "= \"" + name + "\";");
                finalOutput.println(formatIndent(4 * baseIndent) +
                                    formatKey(keysWidth - 2 * baseIndent, ParameterKey.INITIAL_STATE_MANEUVER_NUMBER) +
                                    "= " + state.getManeuversNumber(name) + ";");
                finalOutput.println(formatIndent(4 * baseIndent) +
                                    formatKey(keysWidth - 2 * baseIndent, ParameterKey.INITIAL_STATE_MANEUVER_TOTAL_DV) +
                                    "= " + state.getManeuversTotalDV(name) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) + ((j < maneuversModelsPool.length - 1) ? "}," : "}"));
            }
            finalOutput.println(formatIndent(2 * baseIndent) + "];");

            finalOutput.println(formatIndent(2 * baseIndent) +
                           formatKey(keysWidth, ParameterKey.INITIAL_STATE_ORBIT) +
                           "= {");
            final Orbit orbit = state.getRealState().getOrbit(); 
            finalOutput.println(formatIndent(3 * baseIndent) +
                                formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_TYPE) +
                                "= " + orbit.getType() + ";");
            switch (orbit.getType()) {
            case CARTESIAN :
                final PVCoordinates pv = orbit.getPVCoordinates(inertialFrame);
                final Vector3D p = pv.getPosition();
                final Vector3D v = pv.getVelocity();
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CARTESIAN_DATE) +
                               "= " + orbit.getDate().toString(utc) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CARTESIAN_POSITION) +
                               "= [" + p.getX() + ", " + p.getY() + ", " + p.getZ() + "];");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CARTESIAN_VELOCITY) +
                               "= [" + v.getX() + ", " + v.getY() + ", " + v.getZ() + "];");
                break;
            case KEPLERIAN :
                final KeplerianOrbit kep = (KeplerianOrbit) orbit;
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_DATE) +
                               "= " + orbit.getDate().toString(utc) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_A) +
                               "= " + kep.getA() + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_E) +
                               "= " + kep.getE() + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_I) +
                               "= " + FastMath.toDegrees(kep.getI()) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_PA) +
                               "= " + FastMath.toDegrees(kep.getPerigeeArgument()) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_RAAN) +
                               "= " + FastMath.toDegrees(kep.getRightAscensionOfAscendingNode()) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_KEPLERIAN_ANOMALY) +
                               "= " + FastMath.toDegrees(MathUtils.normalizeAngle(kep.getMeanAnomaly(), FastMath.PI)) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ANGLE_TYPE) +
                               "= " + PositionAngle.MEAN + ";");
               break;
            case CIRCULAR :
                final CircularOrbit cir = (CircularOrbit) orbit;
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_DATE) +
                               "= " + orbit.getDate().toString(utc) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_A) +
                               "= " + cir.getA() + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_EX) +
                               "= " + cir.getCircularEx() + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_EY) +
                               "= " + cir.getCircularEy() + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_I) +
                               "= " + FastMath.toDegrees(cir.getI()) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_RAAN) +
                               "= " + FastMath.toDegrees(cir.getRightAscensionOfAscendingNode()) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_CIRCULAR_LATITUDE_ARGUMENT) +
                               "= " + FastMath.toDegrees(MathUtils.normalizeAngle(cir.getAlphaM(), FastMath.PI)) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ANGLE_TYPE) +
                               "= " + PositionAngle.MEAN + ";");
                break;
            case EQUINOCTIAL :
                final EquinoctialOrbit equ = (EquinoctialOrbit) orbit;
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_DATE) +
                               "= " + orbit.getDate().toString(utc) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_A) +
                               "= " + equ.getA() + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_EX) +
                               "= " + equ.getEquinoctialEx() + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_EY) +
                               "= " + equ.getEquinoctialEy() + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_HX) +
                               "= " + equ.getHx() + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_HY) +
                               "= " + equ.getHy() + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ORBIT_EQUINOCTIAL_LONGITUDE_ARGUMENT) +
                               "= " + FastMath.toDegrees(MathUtils.normalizeAngle(equ.getLM(), FastMath.PI)) + ";");
                finalOutput.println(formatIndent(3 * baseIndent) +
                               formatKey(keysWidth - baseIndent, ParameterKey.ANGLE_TYPE) +
                               "= " + PositionAngle.MEAN + ";");
                break;
            default :
                // this should never happen
                throw SkatException.createInternalError(null);
            }
            finalOutput.println(formatIndent(2 * baseIndent) + "}");

            finalOutput.println(formatIndent(baseIndent) + ((i < finalStates.length - 1) ? "}," : "}"));

        }
        finalOutput.println("];");

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
