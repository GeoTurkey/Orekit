/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.exception.DimensionMismatchException;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.linear.Array2DRowRealMatrix;
import org.apache.commons.math.linear.RealMatrix;
import org.orekit.errors.OrekitException;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.Skat;
import eu.eumetsat.skat.control.ControlLoop;
import eu.eumetsat.skat.control.SKControl;
import eu.eumetsat.skat.realization.ManeuverCrossCoupling;
import eu.eumetsat.skat.realization.ManeuverDateError;
import eu.eumetsat.skat.realization.ManeuverEclipseConstraint;
import eu.eumetsat.skat.realization.ManeuverMagnitudeError;
import eu.eumetsat.skat.realization.ManeuverSplitter;
import eu.eumetsat.skat.realization.MissedManeuver;
import eu.eumetsat.skat.realization.OrbitDetermination;
import eu.eumetsat.skat.realization.Propagation;
import eu.eumetsat.skat.scenario.Scenario;
import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.utils.SupportedPropagator.PropagatorRandomizer;

/** Enumerate for parsing the supported scenario components.
 */
public enum SupportedScenariocomponent {

    /** Constant for embedded scenario. */
    SCENARIO() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final Tree scenarioNode = parser.getValue(node, ParameterKey.SCENARIO);
            final Scenario scenario = new Scenario(skat.getCycleDuration() * Constants.JULIAN_DAY,
                                                   skat.getOutputStep(), skat.getEarth(), skat.getSun(),
                                                   skat.getGroundLocation(),
                                                   skat.getMonitorablesMono(), skat.getMonitorablesDuo(),
                                                   skat.getManeuversMonitorables(),
                                                   skat.getControlLawsMarginsMap(),
                                                   skat.getControlLawsViolationsMap(),
                                                   skat.getManeuversOutput());
            for (int j = 0; j < parser.getElementsNumber(scenarioNode); ++j) {
                final Tree componentNode = parser.getElement(scenarioNode, j);
                final SupportedScenariocomponent component =
                        (SupportedScenariocomponent) parser.getEnumerate(componentNode, ParameterKey.COMPONENT_TYPE,
                                                                         SupportedScenariocomponent.class);
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
                    new Array2DRowRealMatrix(parser.getDoubleArray2(node, ParameterKey.ORBIT_DETERMINATION_CORRELATION),
                                             false);
            if (matrix.getRowDimension() != 6) {
                throw new DimensionMismatchException(matrix.getRowDimension(), 6);
            }
            if (matrix.getColumnDimension() != 6) {
                throw new DimensionMismatchException(matrix.getColumnDimension(), 6);
            }
            double[] standardDeviation = parser.getDoubleArray1(node, ParameterKey.ORBIT_DETERMINATION_STANDARD_DEVIATION);
            if (standardDeviation.length != 6) {
                throw new DimensionMismatchException(matrix.getColumnDimension(), 6);
            }
            return new OrbitDetermination(getIndices(parser, node, skat),
                                          matrix, standardDeviation,
                                          (OrbitType) parser.getEnumerate(node, ParameterKey.ORBIT_TYPE, OrbitType.class),
                                          (PositionAngle) parser.getEnumerate(node, ParameterKey.ANGLE_TYPE, PositionAngle.class),
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
            final int rollingCycles   = skat.getRollingCycles();

            final int firstCycle    = parser.getInt(node, ParameterKey.COMPONENT_CONTROL_LOOP_FIRST_CYCLE);
            final int lastCycle     = parser.getInt(node, ParameterKey.COMPONENT_CONTROL_LOOP_LAST_CYCLE);

            // propagator
            final Tree propagatorNode = parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_PROPAGATOR);
            final  SupportedPropagator sp =
                    (SupportedPropagator) parser.getEnumerate(propagatorNode, ParameterKey.COMPONENT_PROPAGATION_METHOD,
                                                              SupportedPropagator.class);
            final  PropagatorRandomizer propagator =
                    sp.parse(parser, propagatorNode, skat, spacecraftIndex);


            // set up control loop
            final int maxIter = parser.getInt(node, ParameterKey.COMPONENT_CONTROL_LOOP_MAX_ITER);
            final ControlLoop loop = new ControlLoop(spacecraftIndex, firstCycle, lastCycle,
                                                     skat.getManeuversModelsPool(), maxIter, propagator,
                                                     skat.getCycleDuration(), rollingCycles);

            // control laws
            final Tree controlsNode = parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_CONTROLS);
            for (int i = 0; i < parser.getElementsNumber(controlsNode); ++i) {
                final Tree control = parser.getElement(controlsNode, i);
                final SupportedControlLaw sc =
                        (SupportedControlLaw) parser.getEnumerate(control, ParameterKey.CONTROL_TYPE,
                                                                  SupportedControlLaw.class);
                final SKControl controlLaw = sc.parse(parser, control, controlled, skat);
                skat.addControl(controlLaw);
                loop.addControl(controlLaw);
            }

            return loop;

        }
    },

    /** Constant for maneuver_date_error scenario component. */
    MANEUVER_DATE_ERROR() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final String name =
                    parser.getString(node, ParameterKey.COMPONENT_MANEUVER_DATE_ERROR_NAME);
            final double standardDeviation =
                    parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_DATE_ERROR_STANDARD_DEVIATION);
            return new ManeuverDateError(getIndices(parser, node, skat),
                                         name, standardDeviation, skat.getGenerator());
        }
    },

    /** Constant for maneuver magnitude error scenario component. */
    MANEUVER_MAGNITUDE_ERROR() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final String name =
                    parser.getString(node, ParameterKey.COMPONENT_MANEUVER_MAGNITUDE_ERROR_NAME);
            final double standardDeviation =
                    parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_MAGNITUDE_ERROR_STANDARD_DEVIATION);
            return new ManeuverMagnitudeError(getIndices(parser, node, skat),
                                              name, standardDeviation, skat.getGenerator());
        }
    },

    /** Constant for missed maneuver scenario component. */
    MISSED_MANEUVER() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final String name =
                    parser.getString(node, ParameterKey.COMPONENT_MISSED_MANEUVER_NAME);
            final double missThreshold =
                    parser.getDouble(node, ParameterKey.COMPONENT_MISSED_MANEUVER_THRESHOLD);
            final int orbitsSeparation =
                    parser.getInt(node, ParameterKey.COMPONENT_MISSED_MANEUVER_ORBITS_SEPARATION);
            return new MissedManeuver(getIndices(parser, node, skat), name,
                                      missThreshold, orbitsSeparation, skat.getGenerator());
        }
    },

    /** Constant for maneuver coupling scenario component. */
    MANEUVER_CROSS_COUPLING() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final String name =
                    parser.getString(node, ParameterKey.COMPONENT_CROSS_COUPLING_NAME);
            final Vector3D nominalDirection =
                    parser.getVector(node, ParameterKey.COMPONENT_CROSS_COUPLING_NOMINAL_DIRECTION);
            final Vector3D couplingDirection =
                    parser.getVector(node, ParameterKey.COMPONENT_CROSS_COUPLING_COUPLING_DIRECTION);
            final double couplingRatio =
                    parser.getDouble(node, ParameterKey.COMPONENT_CROSS_COUPLING_RATIO);
            return new ManeuverCrossCoupling(getIndices(parser, node, skat), name,
                                             nominalDirection, couplingDirection, couplingRatio);
        }
    },

    /** Constant for maneuver splitting scenario component. */
    MANEUVER_SPLITTER() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final String name =
                    parser.getString(node, ParameterKey.COMPONENT_MANEUVER_SPLITTER_NAME);
            final double maxDV =
                    parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_SPLITTER_MAX_DV);
            final int orbitsSeparation =
                    parser.getInt(node, ParameterKey.COMPONENT_MANEUVER_SPLITTER_ORBITS_SEPARATION);
            return new ManeuverSplitter(getIndices(parser, node, skat), name,
                                        maxDV, orbitsSeparation);
        }
    },

    /** Constant for maneuver splitting scenario component. */
    MANEUVER_ECLIPSE_CONSTRAINT() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final String name =
                    parser.getString(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_NAME);
            final double entryDelay =
                    parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_ENTRY_DELAY);
            final double exitDelay =
                    parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_EXIT_DELAY);
            final int orbitsSeparation =
                    parser.getInt(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_ORBITS_SEPARATION);
            final double minDurationRatio =
                    parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_MIN_DURATION_RATIO);
            return new ManeuverEclipseConstraint(getIndices(parser, node, skat), name,
                                                 entryDelay, exitDelay, orbitsSeparation, minDurationRatio,
                                                 skat.getSun(), skat.getEarth());
        }
    },

    /** Constant for propagation scenario component. */
    PROPAGATION() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {

            final int[] indices = getIndices(parser, node, skat);
            final  PropagatorRandomizer[] randomizers = new PropagatorRandomizer[indices.length];
            for (int i = 0; i < randomizers.length; ++i) {

                // build the propagator randomizer for the selected spacecraft
                final Tree propagatorNode = parser.getValue(node, ParameterKey.COMPONENT_PROPAGATION_PROPAGATOR);
                final SupportedPropagator sp =
                        (SupportedPropagator) parser.getEnumerate(propagatorNode, ParameterKey.COMPONENT_PROPAGATION_METHOD,
                                                                  SupportedPropagator.class);
                randomizers[i] = sp.parse(parser, propagatorNode, skat, i);

            }

            // check if cycle should be truncated after some maneuver
            String truncationManeuverName = null;
            double truncationManeuverDelay = Double.NaN;
            if (parser.containsKey(node, ParameterKey.COMPONENT_PROPAGATION_TRUNCATION_MANEUVER_NAME)) {
                truncationManeuverName =
                        parser.getString(node, ParameterKey.COMPONENT_PROPAGATION_TRUNCATION_MANEUVER_NAME);
                truncationManeuverDelay =
                        parser.getDouble(node, ParameterKey.COMPONENT_PROPAGATION_TRUNCATION_MANEUVER_DELAY);
            }

            // check if long burn inefficiency should be compensated
            final boolean compensateLongBurn = parser.getBoolean(node, ParameterKey.COMPONENT_PROPAGATION_LONG_BURN_COMPENSATION);

            // build the component
            Propagation propagation = new Propagation(indices, randomizers,
                                                      truncationManeuverName, truncationManeuverDelay,
                                                      compensateLongBurn);

            // notify the Skat application this component manages the specified spacecrafts
            for (final int index : indices) {
                if (skat.isManaged(index)) {
                    final SkatException se =
                            new SkatException(SkatMessages.SPACECRAFT_MANAGED_TWICE, skat.getSpacecraftName(index));
                    System.err.println("WARNING: " + se.getLocalizedMessage());
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
