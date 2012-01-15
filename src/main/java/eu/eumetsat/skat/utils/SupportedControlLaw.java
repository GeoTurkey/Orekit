/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Scanner;

import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.exception.util.LocalizedFormats;
import org.apache.commons.math.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.forces.gravity.potential.PotentialCoefficientsProvider;

import eu.eumetsat.skat.Skat;
import eu.eumetsat.skat.control.SKControl;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.strategies.geo.EccentricityCircle;
import eu.eumetsat.skat.strategies.geo.InclinationVector;
import eu.eumetsat.skat.strategies.geo.ParabolicLongitude;
import eu.eumetsat.skat.strategies.leo.GridPoint;
import eu.eumetsat.skat.strategies.leo.GroundTrackGrid;
import eu.eumetsat.skat.strategies.leo.MeanLocalSolarTime;

/** Enumerate for parsing the supported scenario components.
 */
public enum SupportedControlLaw {

    /** Constant for parabolic longitude control law. */
    PARABOLIC_LONGITUDE() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name           = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double sampling       = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double horizon        = parser.getDouble(node, ParameterKey.CONTROL_HORIZON);
            final TunableManeuver model = skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_MANEUVER_NAME));
            final int maxManeuvers      = parser.getInt(node,    ParameterKey.CONTROL_MAX_MANEUVERS);
            final int orbitsSeparation  = parser.getInt(node,    ParameterKey.CONTROL_MANEUVERS_ORBITS_SEPARATION);
            final double firstOffset    = parser.getDouble(node,  ParameterKey.CONTROL_PARABOLIC_FIRST_OFFSET);
            final double lEast          = parser.getAngle(node,  ParameterKey.CONTROL_PARABOLIC_LONGITUDE_EAST);
            final double lWest          = parser.getAngle(node,  ParameterKey.CONTROL_PARABOLIC_LONGITUDE_WEST);
            return new ParabolicLongitude(name, controlled, skat.getSpacecraftIndex(controlled),
                                          model, firstOffset, maxManeuvers, orbitsSeparation,
                                          lEast, lWest, sampling, horizon, skat.getEarth());
        }

    },

    /** Constant for eccentricity circle control law. */
    ECCENTRICITY_CIRCLE() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name           = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double sampling       = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double horizon        = parser.getDouble(node, ParameterKey.CONTROL_HORIZON);
            final TunableManeuver model = skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_MANEUVER_NAME));
            final double centerX        = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_CENTER_X);
            final double centerY        = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_CENTER_Y);
            final double meanRadius     = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_MEAN_RADIUS);
            final double maxRadius      = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_MAX_RADIUS);
            final boolean singleBurn    = parser.getBoolean(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_SINGLE_BURN);
            return new EccentricityCircle(name, controlled, skat.getSpacecraftIndex(controlled),
                                          model, centerX, centerY, meanRadius, maxRadius, singleBurn,
                                          skat.getSun(), sampling, horizon);
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
            final TunableManeuver model   = skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_MANEUVER_NAME));
            final int maxManeuvers        = parser.getInt(node,    ParameterKey.CONTROL_MAX_MANEUVERS);
            final int orbitsSeparation    = parser.getInt(node,    ParameterKey.CONTROL_MANEUVERS_ORBITS_SEPARATION);
            final double firstOffset      = parser.getDouble(node, ParameterKey.CONTROL_INCLINATION_VECTOR_FIRST_OFFSET);
            final double referenceHx      = parser.getDouble(node, ParameterKey.CONTROL_INCLINATION_VECTOR_REFERENCE_HX);
            final double referenceHy      = parser.getDouble(node, ParameterKey.CONTROL_INCLINATION_VECTOR_REFERENCE_HY);
            final double limitInclination = parser.getAngle(node, ParameterKey.CONTROL_INCLINATION_LIMIT_INCLINATION_ANGLE);
            return new InclinationVector(name, controlled, skat.getSpacecraftIndex(controlled),
                                         model, firstOffset, maxManeuvers, orbitsSeparation,
                                         referenceHx, referenceHy, limitInclination, sampling, horizon);
        }

    },

    /** Constant for ground track grid control law. */
    GROUND_TRACK_GRID() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name             = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double horizon          = parser.getDouble(node, ParameterKey.CONTROL_HORIZON);
            final TunableManeuver model   = skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_MANEUVER_NAME));
            final int maxManeuvers        = parser.getInt(node,    ParameterKey.CONTROL_MAX_MANEUVERS);
            final int orbitsSeparation    = parser.getInt(node,    ParameterKey.CONTROL_MANEUVERS_ORBITS_SEPARATION);
            final double firstOffset      = parser.getDouble(node, ParameterKey.CONTROL_GROUND_TRACK_FIRST_OFFSET);
            final double maxDistance      = parser.getDouble(node, ParameterKey.CONTROL_GROUND_TRACK_MAX_CROSS_TRACK_DISTANCE);
            final String fileName         = parser.getString(node, ParameterKey.CONTROL_GROUND_TRACK_GRID_FILE);
            final File gridFile = new File(new File(parser.getInputName()).getParent(), fileName);
            final PotentialCoefficientsProvider gravityField = skat.getgravityField();
            return new GroundTrackGrid(name, controlled, skat.getSpacecraftIndex(controlled),
                                       model, firstOffset, maxManeuvers, orbitsSeparation, skat.getEarth(),
                                       gravityField.getAe(), gravityField.getMu(), gravityField.getJ(false, 2)[2],
                                       readGridFile(gridFile), maxDistance, horizon);
        }

        /** Read a grid file.
         * @param gridFile grid file name
         * @return list of grid points
         * @exception SkatException if file cannot be read
         */
        private List<GridPoint> readGridFile(final File gridFile) throws SkatException {
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
                    grid.add(new GridPoint(timeOffset, latitude, longitude, ascending));

                }

                return grid;

            } catch (IOException ioe) {
                throw new SkatException(ioe, LocalizedFormats.SIMPLE_MESSAGE, ioe.getLocalizedMessage());
            }
        }

    },

    /** Constant for mean local solar time control law. */
    MEAN_LOCAL_SOLAR_TIME() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name               = parser.getString(node,  ParameterKey.CONTROL_NAME);
            final double horizon          = parser.getDouble(node, ParameterKey.CONTROL_HORIZON);
            final TunableManeuver model     = skat.getManeuver(parser.getString(node, ParameterKey.CONTROL_MANEUVER_NAME));
            final int maxManeuvers          = parser.getInt(node,     ParameterKey.CONTROL_MAX_MANEUVERS);
            final int orbitsSeparation      = parser.getInt(node,     ParameterKey.CONTROL_MANEUVERS_ORBITS_SEPARATION);
            final double firstOffset        = parser.getDouble(node,  ParameterKey.CONTROL_SOLAR_TIME_FIRST_OFFSET);
            final double latitude           = parser.getAngle(node,   ParameterKey.CONTROL_SOLAR_TIME_LATITUDE);
            final boolean ascending         = parser.getBoolean(node, ParameterKey.CONTROL_SOLAR_TIME_ASCENDING);
            final double solarTime          = parser.getDouble(node,  ParameterKey.CONTROL_SOLAR_TIME_SOLAR_TIME);
            final double solarTimeTolerance = parser.getDouble(node,  ParameterKey.CONTROL_SOLAR_TIME_SOLAR_TIME_TOLERANCE) / 60.0;
            final PotentialCoefficientsProvider gravityField = skat.getgravityField();
            return new MeanLocalSolarTime(name, controlled, skat.getSpacecraftIndex(controlled),
                                          model, firstOffset, maxManeuvers, orbitsSeparation, skat.getEarth(),
                                          gravityField.getAe(), gravityField.getMu(), gravityField.getJ(false, 2)[2],
                                          latitude, ascending, solarTime, solarTimeTolerance, horizon);
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

}
