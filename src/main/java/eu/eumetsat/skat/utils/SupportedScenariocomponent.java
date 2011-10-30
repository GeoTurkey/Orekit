/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.exception.DimensionMismatchException;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.linear.Array2DRowRealMatrix;
import org.apache.commons.math.linear.RealMatrix;
import org.orekit.errors.OrekitException;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.Propagator;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.Skat;
import eu.eumetsat.skat.control.ControlLoop;
import eu.eumetsat.skat.control.MonitorableDuoSKControl;
import eu.eumetsat.skat.control.MonitorableMonoSKControl;
import eu.eumetsat.skat.control.SKControl;
import eu.eumetsat.skat.realization.ManeuverDateError;
import eu.eumetsat.skat.realization.ManeuverMagnitudeError;
import eu.eumetsat.skat.realization.OrbitDetermination;
import eu.eumetsat.skat.realization.Propagation;
import eu.eumetsat.skat.scenario.Scenario;
import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.strategies.TunableManeuver;

/** Enumerate for parsing the supported scenario components.
 */
public enum SupportedScenariocomponent {

    /** Constant for embedded scenario. */
    SCENARIO() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final Scenario scenario = new Scenario(skat.getCycleDuration() * Constants.JULIAN_DAY,
                                                   skat.getOutputStep(), skat.getEarth(), skat.getSun(),
                                                   skat.getGroundLocation(),
                                                   skat.getMonitorablesMono(), skat.getMonitorablesDuo());
            for (int j = 0; j < parser.getElementsNumber(node); ++j) {
                final Tree componentNode = parser.getElement(node, j);
                final  String type       = parser.getIdentifier(componentNode, ParameterKey.COMPONENT_TYPE);
                final SupportedScenariocomponent component = SupportedScenariocomponent.valueOf(type);
                scenario.addComponent(component.parse(parser, componentNode, skat));
            }
            return scenario;
        }
    },

    /** Constant for orbit determination scenario component. */
    ORBIT_DETERMINATION() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            RealMatrix matrix =
                    new Array2DRowRealMatrix(parser.getDoubleArray2(node, ParameterKey.ORBIT_DETERMINATION_COVARIANCE),
                                             false);
            if (matrix.getRowDimension() != 6) {
                throw new DimensionMismatchException(matrix.getRowDimension(), 6);
            }
            if (matrix.getColumnDimension() != 6) {
                throw new DimensionMismatchException(matrix.getColumnDimension(), 6);
            }
            return new OrbitDetermination(getIndices(parser, node, skat),
                                          matrix,
                                          OrbitType.valueOf(parser.getIdentifier(node, ParameterKey.ORBIT_TYPE)),
                                          parser.getPositionAngle(node, ParameterKey.ANGLE_TYPE),
                                          parser.getDouble(node, ParameterKey.ORBIT_DETERMINATION_SMALL),
                                          skat.getGenerator());
        }
    },

    /** Constant for control loop scenario component. */
    CONTROL_LOOP() {

        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {

            // loop
            final String controlled   = parser.getString(node, ParameterKey.COMPONENT_CONTROL_LOOP_CONTROLLED_SPACECRAFT);
            final int spacecraftIndex = skat.getSpacecraftIndex(controlled);

            final int firstCycle    = parser.getInt(node, ParameterKey.COMPONENT_CONTROL_LOOP_FIRST_CYCLE);
            final int lastCycle     = parser.getInt(node, ParameterKey.COMPONENT_CONTROL_LOOP_LAST_CYCLE);

            // optimizer
            final int maxEval   = parser.getInt(node, ParameterKey.COMPONENT_CONTROL_LOOP_MAX_EVAL);
            final int nbPoints  = parser.getInt(node, ParameterKey.COMPONENT_CONTROL_LOOP_NB_POINTS);
            final String method = parser.getIdentifier(node, ParameterKey.COMPONENT_CONTROL_LOOP_OPTIMIZER);
            final SupportedOptimizer optimizer = SupportedOptimizer.valueOf(method);

            final  Propagator propagator =
                    parser.getPropagator(parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_PROPAGATOR),
                                         skat.getInitialOrbit(spacecraftIndex), skat.getEarth().getBodyFrame());


            final ControlLoop loop = new ControlLoop(spacecraftIndex,
                                                     firstCycle, lastCycle, maxEval, nbPoints,
                                                     optimizer, propagator,
                                                     skat.getCycleDuration(), skat.getRollingCycles());

            // control laws
            final Tree controlsNode = parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_CONTROLS);
            for (int i = 0; i < parser.getElementsNumber(controlsNode); ++i) {
                final Tree control = parser.getElement(controlsNode, i);
                final String type = parser.getIdentifier(control, ParameterKey.CONTROL_TYPE);
                final SKControl controlLaw =
                        SupportedControlLaw.valueOf(type).parse(parser, control, controlled, skat);
                if (controlLaw.getReferenceSpacecraftName() == null) {
                    // this is a control law for a single spacecraft
                    final MonitorableMonoSKControl monitorable = new MonitorableMonoSKControl(controlLaw);
                    skat.addMonitorable(skat.getSpacecraftIndex(controlLaw.getControlledSpacecraftName()),
                                        monitorable);
                    loop.addControl(monitorable);
                } else {
                    // this is a control law for a spacecrafts pair
                    final MonitorableDuoSKControl monitorable = new MonitorableDuoSKControl(controlLaw);
                    skat.addMonitorable(skat.getSpacecraftIndex(controlLaw.getControlledSpacecraftName()),
                                        skat.getSpacecraftIndex(controlLaw.getReferenceSpacecraftName()),
                                        monitorable);
                    loop.addControl(monitorable);
                }
            }

            // tunable maneuvers
            final Tree maneuversNode = parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_MANEUVERS);
            for (int i = 0; i < parser.getElementsNumber(maneuversNode); ++i) {
                final Tree maneuver     = parser.getElement(maneuversNode, i);
                final boolean inPlane    = parser.getBoolean(maneuver, ParameterKey.MANEUVERS_IN_PLANE);
                final String name        = parser.getString(maneuver,  ParameterKey.MANEUVERS_NAME);
                final Vector3D direction = parser.getVector(maneuver,  ParameterKey.MANEUVERS_DIRECTION).normalize();
                final double[][] isp     = parser.getDoubleArray2(maneuver,  ParameterKey.MANEUVERS_ISP_CURVE);
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
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final boolean inPlane =
                    parser.getBoolean(node, ParameterKey.COMPONENT_MANEUVER_DATE_ERROR_IN_PLANE);
            final boolean outOfPlane =
                    parser.getBoolean(node, ParameterKey.COMPONENT_MANEUVER_DATE_ERROR_OUT_OF_PLANE);
            final double standardDeviation =
                    parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_DATE_ERROR_STANDARD_DEVIATION);
            return new ManeuverDateError(getIndices(parser, node, skat),
                                         inPlane, outOfPlane, standardDeviation,
                                         skat.getGenerator());
        }
    },

    /** Constant for maneuver magnitude error scenario component. */
    MANEUVER_MAGNITUDE_ERROR() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final boolean inPlane =
                    parser.getBoolean(node, ParameterKey.COMPONENT_MANEUVER_MAGNITUDE_ERROR_IN_PLANE);
            final boolean outOfPlane =
                    parser.getBoolean(node, ParameterKey.COMPONENT_MANEUVER_MAGNITUDE_ERROR_OUT_OF_PLANE);
            final double standardDeviation =
                    parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_MAGNITUDE_ERROR_STANDARD_DEVIATION);
            return new ManeuverMagnitudeError(getIndices(parser, node, skat),
                                              inPlane, outOfPlane, standardDeviation,
                                              skat.getGenerator());
        }
    },

    /** Constant for propagation scenario component. */
    PROPAGATION() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {

            final int[] indices = getIndices(parser, node, skat);
            final  Propagator[] propagators = new Propagator[indices.length];
            for (int i = 0; i < propagators.length; ++i) {
                propagators[i] = parser.getPropagator(parser.getValue(node, ParameterKey.COMPONENT_PROPAGATION_PROPAGATOR),
                                                      skat.getInitialOrbit(indices[i]),
                                                      skat.getEarth().getBodyFrame());
            }

            // build the component
            Propagation propagation = new Propagation(indices, propagators);

            // notify the Skat application this component manages the specified spacecrafts
            for (final int index : indices) {
                if (skat.isManaged(index)) {
                    throw new SkatException(SkatMessages.SPACECRAFT_MANAGED_TWICE, skat.getSpacecraftName(index));
                }
                skat.manage(index, propagation);
            }

            return propagation;

        }
    };

    /** Parse the indices of the spacecrafts managed by the component.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param skat enclosing Skat tool
     * @return affected indices
     * @exception SkatException if a spacecraft name cannot be recognized
     */
    private static int[] getIndices(final SkatFileParser parser, final Tree node, final Skat skat)
        throws SkatException {
        final Tree namesArrayNode = parser.getValue(node, ParameterKey.COMPONENT_MANAGED_SPACECRAFTS);
        int[] indices = new int[parser.getElementsNumber(namesArrayNode)];
        for (int i = 0; i < indices.length; ++i) {
            indices[i] = skat.getSpacecraftIndex(parser.getElement(namesArrayNode, i).getText());
        }
        return indices;
    }

    /** Parse an input data tree to build a scenario component.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param skat enclosing Skat tool
     * @return parsed component
     * @exception OrekitException if propagator cannot be set up
     * @exception SkatException if control law cannot be recognized
     */
    public abstract ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
        throws OrekitException, SkatException;

}
