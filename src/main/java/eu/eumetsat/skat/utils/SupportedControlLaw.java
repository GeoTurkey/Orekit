/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.antlr.runtime.tree.Tree;
import org.orekit.errors.OrekitException;

import eu.eumetsat.skat.Skat;
import eu.eumetsat.skat.control.SKControl;
import eu.eumetsat.skat.strategies.geo.EccentricityCircle;
import eu.eumetsat.skat.strategies.geo.LongitudeSlotMargins;

/** Enumerate for parsing the supported scenario components.
 */
public enum SupportedControlLaw {

    /** Constant for longitude margins control law. */
    LONGITUDE_MARGINS() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final String name         = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double scale        = parser.getDouble(node, ParameterKey.CONTROL_SCALE);
            final double sampling     = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double eastBoundary = parser.getAngle(node,  ParameterKey.CONTROL_LONGITUDE_MARGINS_EAST);
            final double westBoundary = parser.getAngle(node,  ParameterKey.CONTROL_LONGITUDE_MARGINS_WEST);
            final double target       = parser.getAngle(node,  ParameterKey.CONTROL_LONGITUDE_MARGINS_TARGET);
            return new LongitudeSlotMargins(name, scale, westBoundary, eastBoundary, target, sampling,
                                            skat.getEarth());
        }

    },

    /** Constant for eccentricity circle control law. */
    ECCENTRICITY_CIRCLE() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final String name     = parser.getString(node, ParameterKey.CONTROL_NAME);
            final double sampling = parser.getDouble(node, ParameterKey.CONTROL_SAMPLING);
            final double scale        = parser.getDouble(node, ParameterKey.CONTROL_SCALE);
            final double centerX  = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_CENTER_X);
            final double centerY  = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_CENTER_Y);
            final double radius   = parser.getDouble(node, ParameterKey.CONTROL_ECCENTRICITY_CIRCLE_RADIUS);
            return new EccentricityCircle(name, scale, centerX, centerY, radius, sampling);
        }

    },

    /** Constant for inclination vector secular compensation control law. */
    INCLINATION_VECTOR_SECULAR_COMPENSATION() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            // TODO
            throw SkatException.createInternalError(null);
        }

    },

    /** Constant for relative eccentricity vector control law. */
    RELATIVE_ECCENTRICITY_VECTOR() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            // TODO
            throw SkatException.createInternalError(null);
        }

    },

    /** Constant for relative inclination vector control law. */
    RELATIVE_INCLINATION_VECTOR() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            // TODO
            throw SkatException.createInternalError(null);
        }

    },

    /** Constant for ground track grid control law. */
    GROUND_TRACK_GRID() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            // TODO
            throw SkatException.createInternalError(null);
        }

    },

    /** Constant for mean solar time control law. */
    MEAN_SOLAR_TIME() {

        /** {@inheritDoc} */
        public SKControl parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            // TODO
            throw SkatException.createInternalError(null);
        }

    };

    /** Parse an input data tree to build a control law.
     * @param parser input file parser
     * @param node data node containing control law configuration parameters
     * @param skat enclosing Skat tool
     * @return parsed control law
     * @exception OrekitException if propagator cannot be set up
     * @exception SkatException if control law cannot be recognized
     */
    public abstract SKControl parse(final SkatFileParser parser, final Tree node, final Skat skat)
        throws OrekitException, SkatException;

}
