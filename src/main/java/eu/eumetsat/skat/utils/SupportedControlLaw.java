/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.antlr.runtime.tree.Tree;
import org.orekit.errors.OrekitException;

import eu.eumetsat.skat.Skat;
import eu.eumetsat.skat.control.SKControl;
import eu.eumetsat.skat.strategies.MinimizedManeuvers;
import eu.eumetsat.skat.strategies.geo.EccentricityCircle;
import eu.eumetsat.skat.strategies.geo.CenteredLongitude;
import eu.eumetsat.skat.strategies.geo.InclinationVector;
import eu.eumetsat.skat.strategies.geo.ParabolicLongitude;
import eu.eumetsat.skat.strategies.leo.GroundTrackGrid;
import eu.eumetsat.skat.strategies.leo.MeanLocalSolarTime;

/** Enumerate for parsing the supported scenario components.
 */
public enum SupportedControlLaw {

    /** Constant for centered longitude control law. */
    MINIMIZED_MANEUVERS() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name           = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double scalingDivisor = parser.getDouble(node,  ParameterKey.CONTROL_SCALING_DIVISOR);
            final boolean inPlane       = parser.getBoolean(node, ParameterKey.CONTROL_MINIMIZED_MANEUVERS_IN_PLANE);
            final boolean outOfPlane    = parser.getBoolean(node, ParameterKey.CONTROL_MINIMIZED_MANEUVERS_OUT_OF_PLANE);
            return new MinimizedManeuvers(name, scalingDivisor, controlled, inPlane, outOfPlane);
        }

    },

    /** Constant for centered longitude control law. */
    CENTERED_LONGITUDE() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name           = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double scalingDivisor = parser.getAngle(node,  ParameterKey.CONTROL_SCALING_DIVISOR);
            final double sampling       = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double lEast          = parser.getAngle(node,  ParameterKey.CONTROL_CENTERED_LONGITUDE_EAST);
            final double lWest          = parser.getAngle(node,  ParameterKey.CONTROL_CENTERED_LONGITUDE_WEST);
            return new CenteredLongitude(name, scalingDivisor, controlled, lEast, lWest, sampling, skat.getEarth());
        }

    },

    /** Constant for parabolic longitude control law. */
    PARABOLIC_LONGITUDE() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name                 = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double scalingDivisor       = parser.getAngle(node,  ParameterKey.CONTROL_SCALING_DIVISOR);
            final double sampling             = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double ignoredStartDuration = parser.getDouble(node, ParameterKey.CONTROL_PARABOLIC_IGNORED_START_DURATION);
            final double lEast                = parser.getAngle(node,  ParameterKey.CONTROL_PARABOLIC_LONGITUDE_EAST);
            final double lWest                = parser.getAngle(node,  ParameterKey.CONTROL_PARABOLIC_LONGITUDE_WEST);
            return new ParabolicLongitude(name, scalingDivisor, controlled, ignoredStartDuration,
                                          lEast, lWest, sampling, skat.getEarth());
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
            final double scalingDivisor = parser.getDouble(node, ParameterKey.CONTROL_SCALING_DIVISOR);
            final double centerX        = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_CENTER_X);
            final double centerY        = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_CENTER_Y);
            final double radius         = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_RADIUS);
            return new EccentricityCircle(name, scalingDivisor, controlled, centerX, centerY, radius,
                                          skat.getSun(), sampling);
        }

    },

    /** Constant for inclination vector. */
    INCLINATION_VECTOR() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name           = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double sampling       = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double scalingDivisor = parser.getDouble(node, ParameterKey.CONTROL_SCALING_DIVISOR);
            final double targetHx       = parser.getDouble(node, ParameterKey.CONTROL_INCLINATION_VECTOR_TARGET_X);
            final double targetHy       = parser.getDouble(node, ParameterKey.CONTROL_INCLINATION_VECTOR_TARGET_Y);
            final double circleRadius   = parser.getDouble(node, ParameterKey.CONTROL_INCLINATION_LIMIT_CIRCLE_RADIUS);
            return new InclinationVector(name, scalingDivisor, controlled, targetHx, targetHy, circleRadius, sampling);
        }

    },

    /** Constant for relative eccentricity vector control law. */
    RELATIVE_ECCENTRICITY_VECTOR() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            // TODO
            throw SkatException.createInternalError(null);
        }

    },

    /** Constant for relative inclination vector control law. */
    RELATIVE_INCLINATION_VECTOR() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            // TODO
            throw SkatException.createInternalError(null);
        }

    },

    /** Constant for ground track grid control law. */
    GROUND_TRACK_GRID() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name               = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double scalingDivisor     = parser.getDouble(node, ParameterKey.CONTROL_SCALING_DIVISOR);
            final double latitude           = parser.getAngle(node, ParameterKey.CONTROL_GROUND_TRACK_LATITUDE);
            final double longitude          = parser.getAngle(node, ParameterKey.CONTROL_GROUND_TRACK_LONGITUDE);
            final boolean ascending         = parser.getBoolean(node, ParameterKey.CONTROL_GROUND_TRACK_ASCENDING);
            final int orbitsPerPhasingCycle = parser.getInt(node, ParameterKey.CONTROL_GROUND_TRACK_ORBITS_PER_CYCLE);
            final int daysPerPhasingCycle   = parser.getInt(node, ParameterKey.CONTROL_GROUND_TRACK_DAYS_PER_CYCLE);
            final double minLongitude       = parser.getAngle(node, ParameterKey.CONTROL_GROUND_TRACK_MIN_LONGITUDE);
            final double maxLongitude       = parser.getAngle(node, ParameterKey.CONTROL_GROUND_TRACK_MAX_LONGITUDE);
            return new GroundTrackGrid(name, scalingDivisor, controlled, skat.getEarth(),
                                       latitude, longitude, ascending, orbitsPerPhasingCycle, daysPerPhasingCycle, minLongitude, maxLongitude);
        }

    },

    /** Constant for mean local solar time control law. */
    MEAN_LOCAL_SOLAR_TIME() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name           = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double scalingDivisor = parser.getDouble(node, ParameterKey.CONTROL_SCALING_DIVISOR);
            final double latitude       = parser.getAngle(node, ParameterKey.CONTROL_SOLAR_TIME_LATITUDE);
            final boolean ascending     = parser.getBoolean(node, ParameterKey.CONTROL_SOLAR_TIME_ASCENDING);
            final double solarTime      = parser.getDouble(node, ParameterKey.CONTROL_SOLAR_TIME_SOLAR_TIME);
            final double minSolarTime   = parser.getDouble(node, ParameterKey.CONTROL_SOLAR_TIME_MIN_SOLAR_TIME);
            final double maxSolarTime   = parser.getDouble(node, ParameterKey.CONTROL_SOLAR_TIME_MAX_SOLAR_TIME);
            return new MeanLocalSolarTime(name, scalingDivisor, controlled, skat.getEarth(), latitude,
                                          ascending, solarTime, minSolarTime, maxSolarTime);
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
