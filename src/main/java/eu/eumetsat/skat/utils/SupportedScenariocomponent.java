/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.linear.RealMatrix;
import org.orekit.errors.OrekitException;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;

import eu.eumetsat.skat.Skat;
import eu.eumetsat.skat.control.ControlLoop;
import eu.eumetsat.skat.realization.ManeuverDateError;
import eu.eumetsat.skat.realization.ManeuverMagnitudeError;
import eu.eumetsat.skat.realization.OrbitDetermination;
import eu.eumetsat.skat.realization.Propagation;
import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.strategies.TunableManeuver;

/** Enumerate for parsing the supported scenario components.
 */
public enum SupportedScenariocomponent {

    /** Constant for orbit determination scenario component. */
    ORBIT_DETERMINATION() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node,
                                       final int spacecraftIndex, final Skat skat)
            throws OrekitException, SkatException {
            final Tree covarianceNode =
                    parser.getValue(node, ParameterKey.COMPONENT_ORBIT_DETERMINATION_COVARIANCE);
            final RealMatrix covariance =
                    parser.getCovariance(covarianceNode, ParameterKey.COVARIANCE_MATRIX);
            final PositionAngle positionAngle =
                    parser.getPositionAngle(covarianceNode, ParameterKey.COVARIANCE_ANGLE_TYPE);
            final double small =
                    parser.getDouble(covarianceNode, ParameterKey.COVARIANCE_SMALL);
            return new OrbitDetermination(spacecraftIndex, covariance, positionAngle, small,
                                          skat.getGenerator());
        }
    },

    /** Constant for control loop scenario component. */
    CONTROL_LOOP() {

        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node,
                                       final int spacecraftIndex, final Skat skat)
            throws OrekitException, SkatException {

            // optimizer
            final int maxEval   = parser.getInt(node, ParameterKey.COMPONENT_CONTROL_LOOP_MAX_EVAL);
            final int nbPoints  = parser.getInt(node, ParameterKey.COMPONENT_CONTROL_LOOP_NB_POINTS);
            final String method = parser.getIdentifier(node, ParameterKey.COMPONENT_CONTROL_LOOP_OPTIMIZER);
            final SupportedOptimizer optimizer = SupportedOptimizer.valueOf(method);

            final Propagator propagator = parser.getPropagator(parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_PROPAGATOR),
                                                               skat.getInitialOrbit(spacecraftIndex),
                                                               skat.getEarth().getBodyFrame());


            final ControlLoop loop = new ControlLoop(spacecraftIndex, maxEval, nbPoints, optimizer, propagator);

            // control laws
            final Tree controlsNode = parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_CONTROLS);
            for (int i = 0; i < parser.getElementsNumber(controlsNode); ++i) {
                final Tree control = parser.getElement(controlsNode, i);
                final double scale = parser.getDouble(control, ParameterKey.CONTROL_SCALE);
                final String type = parser.getIdentifier(control, ParameterKey.CONTROL_TYPE);
                SupportedControlLaw law = SupportedControlLaw.valueOf(type);
                loop.addControl(scale, law.parse(parser, control, spacecraftIndex, skat));
            }

            // tunable maneuvers
            final Tree maneuversNode = parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_MANEUVERS);
            for (int i = 0; i < parser.getElementsNumber(maneuversNode); ++i) {
                final Tree maneuver     = parser.getElement(maneuversNode, i);
                final boolean inPlane    = parser.getBoolean(maneuver, ParameterKey.MANEUVERS_IN_PLANE);
                final String name        = parser.getString(maneuver,  ParameterKey.MANEUVERS_NAME);
                final Vector3D direction = parser.getVector(maneuver,  ParameterKey.MANEUVERS_DIRECTION).normalize();
                final double isp         = parser.getDouble(maneuver,  ParameterKey.MANEUVERS_ISP);
                final double dvMin       = parser.getDouble(maneuver,  ParameterKey.MANEUVERS_DV_MIN);
                final double dvMax       = parser.getDouble(maneuver,  ParameterKey.MANEUVERS_DV_MAX);
                final double nominal     = parser.getDouble(maneuver,  ParameterKey.MANEUVERS_NOMINAL_DATE);
                final double dtMin       = parser.getDouble(maneuver,  ParameterKey.MANEUVERS_DT_MIN);
                final double dtMax       = parser.getDouble(maneuver,  ParameterKey.MANEUVERS_DT_MAX);
                loop.addTunableManeuver(new TunableManeuver(name, inPlane, direction, isp,
                                                            dvMin, dvMax, nominal, dtMin, dtMax));
            }

            return loop;

        }
    },

    /** Constant for maneuver_date_error scenario component. */
    MANEUVER_DATE_ERROR() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node,
                                       final int spacecraftIndex, final Skat skat)
            throws OrekitException, SkatException {
            final boolean inPlane =
                    parser.getBoolean(node, ParameterKey.COMPONENT_MANEUVER_DATE_ERROR_IN_PLANE);
            final boolean outOfPlane =
                    parser.getBoolean(node, ParameterKey.COMPONENT_MANEUVER_DATE_ERROR_OUT_OF_PLANE);
            final double standardDeviation =
                    parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_DATE_ERROR_STANDARD_DEVIATION);
            return new ManeuverDateError(spacecraftIndex, inPlane, outOfPlane, standardDeviation,
                                         skat.getGenerator());
        }
    },

    /** Constant for maneuver magnitude error scenario component. */
    MANEUVER_MAGNITUDE_ERROR() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node,
                                       final int spacecraftIndex, final Skat skat)
            throws OrekitException, SkatException {
            final boolean inPlane =
                    parser.getBoolean(node, ParameterKey.COMPONENT_MANEUVER_MAGNITUDE_ERROR_IN_PLANE);
            final boolean outOfPlane =
                    parser.getBoolean(node, ParameterKey.COMPONENT_MANEUVER_MAGNITUDE_ERROR_OUT_OF_PLANE);
            final double standardDeviation =
                    parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_MAGNITUDE_ERROR_STANDARD_DEVIATION);
            return new ManeuverMagnitudeError(spacecraftIndex, inPlane, outOfPlane, standardDeviation,
                                              skat.getGenerator());
        }
    },

    /** Constant for propagation scenario component. */
    PROPAGATION() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node,
                                                final int spacecraftIndex, final Skat skat)
            throws OrekitException, SkatException {
            return new Propagation(parser.getPropagator(parser.getValue(node, ParameterKey.COMPONENT_PROPAGATION_PROPAGATOR),
                                                        skat.getInitialOrbit(spacecraftIndex),
                                                        skat.getEarth().getBodyFrame()));
        }
    };

    /** Parse an input data tree to build a scenario component.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param spacecraftIndex spacecraft index
     * @param skat enclosing Skat tool
     * @return parsed component
     * @exception OrekitException if propagator cannot be set up
     * @exception SkatException if control law cannot be recognized
     */
    public abstract ScenarioComponent parse(final SkatFileParser parser, final Tree node,
                                            final int spacecraftIndex, final Skat skat)
        throws OrekitException, SkatException;

}
