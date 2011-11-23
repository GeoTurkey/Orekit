/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.antlr.runtime.tree.Tree;
import org.orekit.errors.OrekitException;

import eu.eumetsat.skat.Skat;
import eu.eumetsat.skat.control.SKControl;
import eu.eumetsat.skat.strategies.geo.EccentricityCircle;
import eu.eumetsat.skat.strategies.geo.CenteredLongitude;
import eu.eumetsat.skat.strategies.geo.InclinationVector;
import eu.eumetsat.skat.strategies.leo.LocalSolarTime;

/** Enumerate for parsing the supported scenario components.
 */
public enum SupportedControlLaw {

    /** Constant for centered longitude control law. */
    CENTERED_LONGITUDE() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name         = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double scale        = parser.getAngle(node,  ParameterKey.CONTROL_SCALE);
            final double sampling     = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double center       = parser.getAngle(node,  ParameterKey.CONTROL_CENTERED_LONGITUDE_CENTER);
            return new CenteredLongitude(name, scale, controlled, center, sampling, skat.getEarth());
        }

    },

    /** Constant for eccentricity circle control law. */
    ECCENTRICITY_CIRCLE() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name         = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double sampling     = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double scale        = parser.getDouble(node, ParameterKey.CONTROL_SCALE);
            final double centerX      = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_CENTER_X);
            final double centerY      = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_CENTER_Y);
            final double radius       = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_RADIUS);
            return new EccentricityCircle(name, scale, controlled, centerX, centerY, radius, sampling);
        }

    },

    /** Constant for inclination vector. */
    INCLINATION_VECTOR() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name         = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double sampling     = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double scale        = parser.getDouble(node, ParameterKey.CONTROL_SCALE);
            final double targetHx     = parser.getDouble(node, ParameterKey.CONTROL_INCLINATION_VECTOR_TARGET_X);
            final double targetHy     = parser.getDouble(node, ParameterKey.CONTROL_INCLINATION_VECTOR_TARGET_Y);
            final double circleRadius = parser.getDouble(node, ParameterKey.CONTROL_INCLINATION_LIMIT_CIRCLE_RADIUS);
            return new InclinationVector(name, scale, controlled, targetHx, targetHy, circleRadius, sampling);
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
            // TODO
            throw SkatException.createInternalError(null);
        }

    },

    /** Constant for local solar time control law. */
    LOCAL_SOLAR_TIME() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node,
                               final String controlled, final Skat skat)
            throws OrekitException, SkatException {
            final String name         = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double scale        = parser.getDouble(node, ParameterKey.CONTROL_SCALE);
            final double latitude     = parser.getDouble(node, ParameterKey.CONTROL_SOLAR_TIME_LATITUDE);
            final boolean ascending   = parser.getBoolean(node, ParameterKey.CONTROL_SOLAR_TIME_ASCENDING);
            final double solarTime    = parser.getDouble(node, ParameterKey.CONTROL_SOLAR_TIME_SOLAR_TIME);
            return new LocalSolarTime(name, scale, controlled, skat.getEarth(), skat.getSun(),
                                      latitude, ascending, solarTime);
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
