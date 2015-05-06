/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Scanner;

import org.antlr.runtime.tree.Tree;
import org.apache.commons.math3.exception.util.LocalizedFormats;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.UnnormalizedSphericalHarmonicsProvider;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.DateTimeComponents;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.Skat;
import eu.eumetsat.skat.control.SKControl;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.strategies.geo.EccentricityCircle;
import eu.eumetsat.skat.strategies.geo.InclinationVector;
import eu.eumetsat.skat.strategies.geo.ParabolicLongitude;
import eu.eumetsat.skat.strategies.leo.GridPoint;
import eu.eumetsat.skat.strategies.leo.InPlaneGroundTrackGrid;
import eu.eumetsat.skat.strategies.leo.MeanLocalSolarTime;
import eu.eumetsat.skat.strategies.leo.Inclination;
import eu.eumetsat.skat.strategies.leo.OutOfPlaneGroundTrackGrid;

/** Enumerate for parsing the supported scenario components.
 */
public enum SupportedControlLaw {

    /** Constant for parabolic longitude control law. */
    PARABOLIC_LONGITUDE() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name              = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double sampling          = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double horizon           = parser.getDouble(node, ParameterKey.CONTROL_HORIZON);
            TunableManeuver[] model;
            final Tree namesArrayNode      = parser.getValue(node, ParameterKey.CONTROL_MANEUVER_NAME);
            if(namesArrayNode.getType() == SkatParser.ARRAY){
            	model = new TunableManeuver[parser.getElementsNumber(namesArrayNode)];
            	for (int i = 0; i < model.length; ++i) {
            		model[i] = skat.getManeuver(parser.getElement(namesArrayNode, i).getText());
            	}
            }
            else{
            	model    = new TunableManeuver[1];
            	model[0] = skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_MANEUVER_NAME));
            }
            final int[][] yawFlipSequence  = {};
            final int maxManeuvers         = parser.getInt(node,    ParameterKey.CONTROL_MAX_MANEUVERS);
            final int orbitsSeparation     = parser.getInt(node,    ParameterKey.CONTROL_MANEUVERS_ORBITS_SEPARATION);
            final double firstOffset       = parser.getDouble(node,  ParameterKey.CONTROL_PARABOLIC_FIRST_OFFSET);
            final double lEast             = parser.getAngle(node,  ParameterKey.CONTROL_PARABOLIC_LONGITUDE_EAST);
            final double lWest             = parser.getAngle(node,  ParameterKey.CONTROL_PARABOLIC_LONGITUDE_WEST);
            return new ParabolicLongitude(name, controlled, skat.getSpacecraftIndex(controlled),
                                          model, yawFlipSequence, firstOffset, maxManeuvers, orbitsSeparation,
                                          lEast, lWest, sampling, horizon, skat.getEarth());
        }

    },

    /** Constant for eccentricity circle control law. */
    ECCENTRICITY_CIRCLE() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name             = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double sampling         = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double horizon          = parser.getDouble(node, ParameterKey.CONTROL_HORIZON);
            TunableManeuver[] model;
            final Tree namesArrayNode     = parser.getValue(node, ParameterKey.CONTROL_MANEUVER_NAME);
            if(namesArrayNode.getType() == SkatParser.ARRAY){
            	model = new TunableManeuver[parser.getElementsNumber(namesArrayNode)];
            	for (int i = 0; i < model.length; ++i) {
            		model[i] = skat.getManeuver(parser.getElement(namesArrayNode, i).getText());
            	}
            }
            else{
            	model    = new TunableManeuver[1];
            	model[0] = skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_MANEUVER_NAME));
            }
            final int[][] yawFlipSequence     = {}; 
            final double centerX              = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_CENTER_X);
            final double centerY              = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_CENTER_Y);
            final double meanRadius           = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_MEAN_RADIUS);
            final double maxRadius            = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_MAX_RADIUS);
            final boolean singleBurn          = parser.getBoolean(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_SINGLE_BURN);
            final TunableManeuver oopManeuver = parser.containsKey(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_OOP_MANEUVER_NAME) ? 
            		skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_OOP_MANEUVER_NAME)) : null;
            
            return new EccentricityCircle(name, controlled, skat.getSpacecraftIndex(controlled),
                                          model, yawFlipSequence, centerX, centerY, meanRadius, maxRadius, singleBurn,
                                          skat.getSun(), sampling, horizon, oopManeuver);
        }

    },

    /** Constant for inclination vector. */
    INCLINATION_VECTOR() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name             = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double sampling         = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double horizon          = parser.getDouble(node, ParameterKey.CONTROL_HORIZON);
            TunableManeuver[] model;
            final Tree namesArrayNode   = parser.getValue(node, ParameterKey.CONTROL_MANEUVER_NAME);
            if(namesArrayNode.getType() == SkatParser.ARRAY){
            	model = new TunableManeuver[parser.getElementsNumber(namesArrayNode)];
            	for (int i = 0; i < model.length; ++i) {
            		model[i] = skat.getManeuver(parser.getElement(namesArrayNode, i).getText());
            	}
            }
            else{
            	model    = new TunableManeuver[1];
            	model[0] = skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_MANEUVER_NAME));
            }
            int[][] emptyArray = {};
            final int[][] yawFlipSequence =  
                    parser.containsKey(node, ParameterKey.CONTROL_YAW_FLIP_SEQUENCE) ? 
                    parser.getIntArray2(node, ParameterKey.CONTROL_YAW_FLIP_SEQUENCE) : emptyArray;
            final int maxManeuvers        = parser.getInt(node,    ParameterKey.CONTROL_MAX_MANEUVERS);
            final int orbitsSeparation    = parser.getInt(node,    ParameterKey.CONTROL_MANEUVERS_ORBITS_SEPARATION);
            final double firstOffset      = parser.getDouble(node, ParameterKey.CONTROL_INCLINATION_VECTOR_FIRST_OFFSET);
            final double referenceHx      = parser.getDouble(node, ParameterKey.CONTROL_INCLINATION_VECTOR_REFERENCE_HX);
            final double referenceHy      = parser.getDouble(node, ParameterKey.CONTROL_INCLINATION_VECTOR_REFERENCE_HY);
            final double limitInclination = parser.getAngle(node, ParameterKey.CONTROL_INCLINATION_LIMIT_INCLINATION_ANGLE);
            final boolean isPairedManeuvers = parser.containsKey(node, ParameterKey.CONTROL_INCLINATION_IS_PAIRED_MANEUVERS) ? parser.getBoolean(node,ParameterKey.CONTROL_INCLINATION_IS_PAIRED_MANEUVERS) : false;
            return new InclinationVector(name, controlled, skat.getSpacecraftIndex(controlled),
                                         model, yawFlipSequence, firstOffset, maxManeuvers, orbitsSeparation,
                                         referenceHx, referenceHy, limitInclination, sampling, horizon, isPairedManeuvers);
        }

    },

    /** Constant for in-plane ground track grid control law. */
    IN_PLANE_GROUND_TRACK_GRID() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name             = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double horizon          = parser.getDouble(node, ParameterKey.CONTROL_HORIZON);
            TunableManeuver[] model;
            final Tree namesArrayNode   = parser.getValue(node, ParameterKey.CONTROL_MANEUVER_NAME);
            if(namesArrayNode.getType() == SkatParser.ARRAY){
            	model = new TunableManeuver[parser.getElementsNumber(namesArrayNode)];
            	for (int i = 0; i < model.length; ++i) {
            		model[i] = skat.getManeuver(parser.getElement(namesArrayNode, i).getText());
            	}
            }
            else{
            	model    = new TunableManeuver[1];
            	model[0] = skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_MANEUVER_NAME));
            }
            final int[][] yawFlipSequence            =  {};
            final int maxManeuvers                   = parser.getInt(node,     ParameterKey.CONTROL_MAX_MANEUVERS);
            final int orbitsSeparation               = parser.getInt(node,     ParameterKey.CONTROL_MANEUVERS_ORBITS_SEPARATION);
            final double firstOffset                 = parser.getDouble(node,  ParameterKey.CONTROL_IN_PLANE_GROUND_TRACK_FIRST_OFFSET);
            final double maxDistance                 = parser.getDouble(node,  ParameterKey.CONTROL_IN_PLANE_GROUND_TRACK_MAX_CROSS_TRACK_DISTANCE);
            final String fileName                    = parser.getString(node,  ParameterKey.CONTROL_IN_PLANE_GROUND_TRACK_GRID_FILE);
            final double inclinationOffsetFineTuning = parser.containsKey(node,ParameterKey.CONTROL_IN_PLANE_GROUND_TRACK_GRID_INCLINATION_OFFSET_FINE_TUNING) ? 
                                                       parser.getDouble(node,  ParameterKey.CONTROL_IN_PLANE_GROUND_TRACK_GRID_INCLINATION_OFFSET_FINE_TUNING) : Double.NaN;
            final File gridFile                      = new File(new File(parser.getInputName()).getParent(), fileName);
            final UnnormalizedSphericalHarmonicsProvider gravityField = GravityFieldFactory.getUnnormalizedProvider(2, 0);
            return new InPlaneGroundTrackGrid(name, controlled, skat.getSpacecraftIndex(controlled),
                                              model, yawFlipSequence, firstOffset, maxManeuvers, orbitsSeparation,
                                              skat.getEarth(), skat.getSun(),
                                              gravityField.getAe(), gravityField.getMu(),
                                              -gravityField.onDate(AbsoluteDate.J2000_EPOCH).getUnnormalizedCnm(2, 0),
                                              readGridFile(gridFile, skat.getEarth()), maxDistance, horizon, inclinationOffsetFineTuning);
        }

    },

    /** Constant for out-of-plane ground track grid control law. */
    OUT_OF_PLANE_GROUND_TRACK_GRID() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name             = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double horizon          = parser.getDouble(node, ParameterKey.CONTROL_HORIZON);
            TunableManeuver[] model;
            final Tree namesArrayNode   = parser.getValue(node, ParameterKey.CONTROL_MANEUVER_NAME);
            if(namesArrayNode.getType() == SkatParser.ARRAY){
            	model = new TunableManeuver[parser.getElementsNumber(namesArrayNode)];
            	for (int i = 0; i < model.length; ++i) {
            		model[i] = skat.getManeuver(parser.getElement(namesArrayNode, i).getText());
            	}
            }
            else{
            	model    = new TunableManeuver[1];
            	model[0] = skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_MANEUVER_NAME));
            }
            final int[][] yawFlipSequence            = {};
            final int maxManeuvers                   = parser.getInt(node,      ParameterKey.CONTROL_MAX_MANEUVERS);
            final int orbitsSeparation               = parser.getInt(node,      ParameterKey.CONTROL_MANEUVERS_ORBITS_SEPARATION);
            final double firstOffset                 = parser.getDouble(node,   ParameterKey.CONTROL_OUT_OF_PLANE_GROUND_TRACK_FIRST_OFFSET);
            final double maxDistance                 = parser.getDouble(node,   ParameterKey.CONTROL_OUT_OF_PLANE_GROUND_TRACK_MAX_CROSS_TRACK_DISTANCE);
            final String fileName                    = parser.getString(node,   ParameterKey.CONTROL_OUT_OF_PLANE_GROUND_TRACK_GRID_FILE);
            final boolean compensateLongBurn         = parser.getBoolean(node,  ParameterKey.CONTROL_OUT_OF_PLANE_GROUND_TRACK_GRID_BURN_COMPENSATION);
            final double inclinationOffsetFineTuning = parser.containsKey(node, ParameterKey.CONTROL_OUT_OF_PLANE_GROUND_TRACK_GRID_INCLINATION_OFFSET_FINE_TUNING) ? 
            		                                   parser.getDouble(node,   ParameterKey.CONTROL_OUT_OF_PLANE_GROUND_TRACK_GRID_INCLINATION_OFFSET_FINE_TUNING) : Double.NaN;
            final File gridFile                      = new File(new File(parser.getInputName()).getParent(), fileName);
            final UnnormalizedSphericalHarmonicsProvider gravityField = GravityFieldFactory.getUnnormalizedProvider(2, 0);
            return new OutOfPlaneGroundTrackGrid(name, controlled, skat.getSpacecraftIndex(controlled),
                                                 model, yawFlipSequence, firstOffset, maxManeuvers, orbitsSeparation,
                                                 skat.getEarth(), skat.getSun(),
                                                 gravityField.getAe(), gravityField.getMu(),
                                                 -gravityField.onDate(AbsoluteDate.J2000_EPOCH).getUnnormalizedCnm(2, 0),
                                                 readGridFile(gridFile, skat.getEarth()), maxDistance, horizon,
                                                 compensateLongBurn, inclinationOffsetFineTuning);
        }

    },

    /** Constant for inclination control law. */
    INCLINATION() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name                = parser.getString(node,    ParameterKey.CONTROL_NAME);
            final double horizon             = parser.getDouble(node,    ParameterKey.CONTROL_HORIZON);
            TunableManeuver[] model;
            final Tree namesArrayNode   = parser.getValue(node, ParameterKey.CONTROL_MANEUVER_NAME);
            if(namesArrayNode.getType() == SkatParser.ARRAY){
            	model = new TunableManeuver[parser.getElementsNumber(namesArrayNode)];
            	for (int i = 0; i < model.length; ++i) {
            		model[i] = skat.getManeuver(parser.getElement(namesArrayNode, i).getText());
            	}
            }
            else{
            	model    = new TunableManeuver[1];
            	model[0] = skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_MANEUVER_NAME));
            }
            final int[][] yawFlipSequence    = {};  
            final int maxManeuvers           = parser.getInt(node,       ParameterKey.CONTROL_MAX_MANEUVERS);
            final int orbitsSeparation       = parser.getInt(node,       ParameterKey.CONTROL_MANEUVERS_ORBITS_SEPARATION);
            final double firstOffset         = parser.getDouble(node,    ParameterKey.CONTROL_INCLINATION_FIRST_OFFSET);
            final double incMeanValue        = parser.getAngle(node,    ParameterKey.CONTROL_INCLINATION_MEAN_VALUE);
            final double incDeadband         = parser.getAngle(node,    ParameterKey.CONTROL_INCLINATION_TOLERANCE);
            final boolean compensateLongBurn = parser.getBoolean(node,   ParameterKey.CONTROL_INCLINATION_LONG_BURN_COMPENSATION);
            final double iDrift0             = parser.getAngle(node,     ParameterKey.CONTROL_INCLINATION_I_DRIFT_0)   / Constants.JULIAN_DAY;
            final double iDriftCos           = parser.getAngle(node,     ParameterKey.CONTROL_INCLINATION_I_DRIFT_COS) / Constants.JULIAN_DAY;
            final double iDriftDoy           = parser.getDouble(node,    ParameterKey.CONTROL_INCLINATION_I_DRIFT_DOY);
            final int phasingDays            = parser.getInt(node,       ParameterKey.CONTROL_INCLINATION_PHASING_CYCLE_DAYS);
            final int phasingOrbits          = parser.getInt(node,       ParameterKey.CONTROL_INCLINATION_PHASING_CYCLE_ORBITS);
            final int[] maneuversDoy         = parser.getIntArray1(node, ParameterKey.CONTROL_INCLINATION_MANEUVERS_DOY);
            final UnnormalizedSphericalHarmonicsProvider gravityField = GravityFieldFactory.getUnnormalizedProvider(2, 0);
            final TimeScale utc = TimeScalesFactory.getUTC();

            // inclination offset function in radians
            final Inclination.InclinationModel analyticalModels =
                    new Inclination.InclinationModel() {

                /** {@inheritDoc} */
                public boolean increasingInclination() {
                    return iDrift0 >= 0;
                }

                /** {@inheritDoc} */
                public double inc(final AbsoluteDate t, final AbsoluteDate t0, final double inc0, final double iOffsetRef) {
                    final double dt        = t.durationFrom(t0);
                    final double pulsation = 2 * FastMath.PI / (365 * Constants.JULIAN_DAY);
                    final double sin0      = FastMath.sin(pulsation * timeOffset(t0));
                    final double sinT      = FastMath.sin(pulsation * timeOffset(t));
                    final double deltaI    = iOffsetRef + iDrift0 * dt -
                                             iDriftCos / pulsation * (sinT - sin0) ;
                    return inc0 + deltaI;
                }

                /** Compute time offset.
                 * @param date date
                 * @return time offset at specified date
                 */
                private double timeOffset(final AbsoluteDate date) {
                    final DateTimeComponents dtc = date.getComponents(utc);
                    final int doy        = dtc.getDate().getDayOfYear();
                    final double seconds = dtc.getTime().getSecondsInDay();
                    return (doy - iDriftDoy) * Constants.JULIAN_DAY + seconds;
                }

            };

            return new Inclination(name, controlled, skat.getSpacecraftIndex(controlled),
                                          model, yawFlipSequence, firstOffset, maxManeuvers, orbitsSeparation,
                                          skat.getEarth(), skat.getSun(),
                                          gravityField.getAe(), gravityField.getMu(),
                                          -gravityField.onDate(AbsoluteDate.J2000_EPOCH).getUnnormalizedCnm(2, 0),
                                          incMeanValue, incDeadband, horizon, compensateLongBurn,
                                          phasingDays, phasingOrbits, maneuversDoy, analyticalModels);

        }

    },

    /** Constant for mean local solar time control law. */
    MEAN_LOCAL_SOLAR_TIME() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name                = parser.getString(node,    ParameterKey.CONTROL_NAME);
            final double horizon             = parser.getDouble(node,    ParameterKey.CONTROL_HORIZON);
            TunableManeuver[] model;
            final Tree namesArrayNode   = parser.getValue(node, ParameterKey.CONTROL_MANEUVER_NAME);
            if(namesArrayNode.getType() == SkatParser.ARRAY){
            	model = new TunableManeuver[parser.getElementsNumber(namesArrayNode)];
            	for (int i = 0; i < model.length; ++i) {
            		model[i] = skat.getManeuver(parser.getElement(namesArrayNode, i).getText());
            	}
            }
            else{
            	model    = new TunableManeuver[1];
            	model[0] = skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_MANEUVER_NAME));
            }
            final int[][] yawFlipSequence    =  {};
            final int maxManeuvers           = parser.getInt(node,       ParameterKey.CONTROL_MAX_MANEUVERS);
            final int orbitsSeparation       = parser.getInt(node,       ParameterKey.CONTROL_MANEUVERS_ORBITS_SEPARATION);
            final double firstOffset         = parser.getDouble(node,    ParameterKey.CONTROL_SOLAR_TIME_FIRST_OFFSET);
            final double latitude            = parser.getAngle(node,     ParameterKey.CONTROL_SOLAR_TIME_LATITUDE);
            final boolean ascending          = parser.getBoolean(node,   ParameterKey.CONTROL_SOLAR_TIME_ASCENDING);
            final double solarTime           = parser.getDouble(node,    ParameterKey.CONTROL_SOLAR_TIME_SOLAR_TIME);
            final double solarTimeTolerance  = parser.getDouble(node,    ParameterKey.CONTROL_SOLAR_TIME_SOLAR_TIME_TOLERANCE) / 60.0;
            final boolean compensateLongBurn = parser.getBoolean(node,   ParameterKey.CONTROL_SOLAR_TIME_LONG_BURN_COMPENSATION);
            final double iDrift0             = parser.getAngle(node,     ParameterKey.CONTROL_SOLAR_TIME_I_DRIFT_0)   / Constants.JULIAN_DAY;
            final double iDriftCos           = parser.getAngle(node,     ParameterKey.CONTROL_SOLAR_TIME_I_DRIFT_COS) / Constants.JULIAN_DAY;
            final double iDriftDoy           = parser.getDouble(node,    ParameterKey.CONTROL_SOLAR_TIME_I_DRIFT_DOY);
            final int phasingDays            = parser.getInt(node,       ParameterKey.CONTROL_SOLAR_TIME_PHASING_CYCLE_DAYS);
            final int phasingOrbits          = parser.getInt(node,       ParameterKey.CONTROL_SOLAR_TIME_PHASING_CYCLE_ORBITS);
            final int[] maneuversDoy         = parser.getIntArray1(node, ParameterKey.CONTROL_SOLAR_TIME_MANEUVERS_DOY);
            final UnnormalizedSphericalHarmonicsProvider gravityField = GravityFieldFactory.getUnnormalizedProvider(2, 0);
            final TimeScale utc = TimeScalesFactory.getUTC();

            // inclination offset function in radians
            final MeanLocalSolarTime.MlstModel analyticalModels =
                    new MeanLocalSolarTime.MlstModel() {

                /** {@inheritDoc} */
                public boolean increasingInclination() {
                    return iDrift0 >= 0;
                }

                /** {@inheritDoc} */
                public double mlst(final AbsoluteDate t, final AbsoluteDate tRef, final double dhDotDi,
                                   final double mlstRef, final double iOffsetRef) {
                    final double dt        = t.durationFrom(tRef);
                    final double pulsation = 2 * FastMath.PI / (365 * Constants.JULIAN_DAY);
                    final double cos0      = FastMath.cos(pulsation * timeOffset(tRef));
                    final double cosT      = FastMath.cos(pulsation * timeOffset(t));
                    final double deltaI    = iOffsetRef * dt + 0.5 * iDrift0 * dt * dt -
                                             iDriftCos * (cosT - cos0) / (pulsation * pulsation);
                    return mlstRef + dhDotDi * deltaI;
                }

                /** Compute time offset.
                 * @param date date
                 * @return time offset at specified date
                 */
                private double timeOffset(final AbsoluteDate date) {
                    final DateTimeComponents dtc = date.getComponents(utc);
                    final int doy        = dtc.getDate().getDayOfYear();
                    final double seconds = dtc.getTime().getSecondsInDay();
                    return (doy - iDriftDoy) * Constants.JULIAN_DAY + seconds;
                }

            };

            return new MeanLocalSolarTime(name, controlled, skat.getSpacecraftIndex(controlled),
                                          model, yawFlipSequence, firstOffset, maxManeuvers, orbitsSeparation,
                                          skat.getEarth(), skat.getSun(),
                                          gravityField.getAe(), gravityField.getMu(),
                                          -gravityField.onDate(AbsoluteDate.J2000_EPOCH).getUnnormalizedCnm(2, 0),
                                          latitude, ascending, solarTime, solarTimeTolerance, horizon, compensateLongBurn,
                                          phasingDays, phasingOrbits, maneuversDoy, analyticalModels);

        }

    };

    /** Parse an input data tree to build a control law.
     * @param parser input file parser
     * @param node data node containing control law configuration parameters
     * @param controlled name of the controlled spacecraft
     * @param skat enclosing Skat tool
     * @return parsed control law
     * @exception OrekitException if propagator cannot be set up
     * @exception SkatException if control law cannot be recognized
     */
    public abstract SKControl parse(final SkatFileParser parser, final Tree node,
                                    final String controlled, final Skat skat)
        throws OrekitException, SkatException;

    /** Read a grid file.
     * @param gridFile grid file name
     * @param earth Earth model
     * @return list of grid points
     * @exception SkatException if file cannot be read
     */
    protected List<GridPoint> readGridFile(final File gridFile, final OneAxisEllipsoid earth)
        throws SkatException {
        try {

            if (! gridFile.exists()) {
                throw new SkatException(SkatMessages.UNABLE_TO_FIND_RESOURCE, gridFile.getAbsolutePath());
            }
            final Scanner scanner = new Scanner(gridFile).useLocale(Locale.US);

            List<GridPoint> grid = new ArrayList<GridPoint>();

            while (scanner.hasNextDouble()) {

                // read one grid point per line
                final double  timeOffset = scanner.nextDouble();
                final double  latitude   = FastMath.toRadians(scanner.nextDouble());
                final double  longitude  = FastMath.toRadians(scanner.nextDouble());
                final boolean ascending  = scanner.nextBoolean();
                grid.add(new GridPoint(timeOffset, latitude, longitude, earth, ascending));

            }

            return grid;

        } catch (IOException ioe) {
            throw new SkatException(ioe, LocalizedFormats.SIMPLE_MESSAGE, ioe.getLocalizedMessage());
        }
    }

}
