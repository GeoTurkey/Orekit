/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.antlr.runtime.tree.Tree;
import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.orekit.errors.OrekitException;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;

import eu.eumetsat.skat.Skat;
import eu.eumetsat.skat.control.ControlLoop;
import eu.eumetsat.skat.control.SKControl;
import eu.eumetsat.skat.realization.ManeuverCrossCoupling;
import eu.eumetsat.skat.realization.ManeuverDateError;
import eu.eumetsat.skat.realization.ManeuverEclipseConstraint;
import eu.eumetsat.skat.realization.ManeuverMagnitudeError;
import eu.eumetsat.skat.realization.MissedManeuver;
import eu.eumetsat.skat.realization.OrbitDetermination;
import eu.eumetsat.skat.realization.Propagation;
import eu.eumetsat.skat.scenario.Scenario;
import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SupportedPropagator.PropagatorRandomizer;

import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;

/** Enumerate for parsing the supported scenario components.
 */
public enum SupportedScenariocomponent {

    /** Constant for embedded scenario. */
    SCENARIO() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final Tree scenarioNode = parser.getValue(node, ParameterKey.SCENARIO);
            final Scenario scenario = new Scenario(skat.getOutputStep(),
                                                   skat.getEarth(), skat.getSun(),
                                                   skat.getGroundLocation(),
                                                   skat.getMonitorablesMono(), skat.getMonitorablesDuo(),
                                                   skat.getManeuversMonitorables(),
                                                   skat.getControlLawsValuesMap(),
                                                   skat.getControlLawsViolationsMap(),
                                                   skat.getManeuversOutput());
            for (int j = 0; j < parser.getElementsNumber(scenarioNode); ++j) {
                final Tree componentNode = parser.getElement(scenarioNode, j);
                final SupportedScenariocomponent component =
                        parser.getEnumerate(componentNode, ParameterKey.COMPONENT_TYPE, SupportedScenariocomponent.class);
                scenario.addComponent(component.parse(parser, componentNode, skat));
            }
            // Set cycle duration and check (if necessary) the time horizon to cycle duration ratio
            final double timeHor2cycleDur = 2.0;
            scenario.setCycleDuration(skat.getCycleDuration(),skat.getCheckTimeHorizons(),timeHor2cycleDur);
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
                                          parser.getEnumerate(node, ParameterKey.ORBIT_TYPE, OrbitType.class),
                                          parser.getEnumerate(node, ParameterKey.ANGLE_TYPE, PositionAngle.class),
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

            // propagator
            final Tree propagatorNode = parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_PROPAGATOR);
            final  SupportedPropagator sp =
                    parser.getEnumerate(propagatorNode, ParameterKey.COMPONENT_PROPAGATION_METHOD,
                                                              SupportedPropagator.class);
            final  PropagatorRandomizer propagator =
                    sp.parse(parser, propagatorNode, skat, spacecraftIndex);


            // set up control loop
            final int maxIter = parser.getInt(node, ParameterKey.COMPONENT_CONTROL_LOOP_MAX_ITER);
            final ControlLoop loop = new ControlLoop(spacecraftIndex, firstCycle, lastCycle, maxIter, propagator, skat.getEndDate());

            // control laws
            final Tree controlsNode = parser.getValue(node, ParameterKey.COMPONENT_CONTROL_LOOP_CONTROLS);
            for (int i = 0; i < parser.getElementsNumber(controlsNode); ++i) {
                final Tree control = parser.getElement(controlsNode, i);
                final SupportedControlLaw sc =
                        parser.getEnumerate(control, ParameterKey.CONTROL_TYPE, SupportedControlLaw.class);
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
            // Get some general simulation parameters
            TimeScale utc = TimeScalesFactory.getUTC();
            final Tree simulationNode = parser.getValue(parser.getRoot(), ParameterKey.SIMULATION);
            AbsoluteDate startDate = 
                    parser.getDate(simulationNode, ParameterKey.SIMULATION_START_DATE, utc);
            
            final String name =
                    parser.getString(node, ParameterKey.COMPONENT_CROSS_COUPLING_NAME);
            final Vector3D nominalDirection =
                    parser.getVector(node, ParameterKey.COMPONENT_CROSS_COUPLING_NOMINAL_DIRECTION);

            final double standardDeviation1 =
                    parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_STANDARD_DEVIATION1) ?
                    parser.getDouble(node, ParameterKey.COMPONENT_CROSS_COUPLING_STANDARD_DEVIATION1) : 0.0;
            final Vector3D couplingDirection1 =
                    parser.getVector(node, ParameterKey.COMPONENT_CROSS_COUPLING_DIRECTION_1);
            final double constantCouplingRatio1 =
                    parser.getDouble(node, ParameterKey.COMPONENT_CROSS_COUPLING_CONSTANT_RATIO_1);
            final double secularVariation1 =
                    parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_SECULAR_VARIATION_1) ? 
                    parser.getDouble(node, ParameterKey.COMPONENT_CROSS_COUPLING_SECULAR_VARIATION_1) : 0.0;
            final AbsoluteDate secularZero1 =
                    parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_SECULAR_ZERO_1) ? 
                    parser.getDate(node, ParameterKey.COMPONENT_CROSS_COUPLING_SECULAR_ZERO_1,utc) : startDate;
            final double period1 =
                    parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_PERIOD_1) ? 
                    parser.getDouble(node, ParameterKey.COMPONENT_CROSS_COUPLING_PERIOD_1) : 0.0;
            final AbsoluteDate harmonicZero1 =
                    parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_PERIOD_ZERO_1) ? 
                    parser.getDate(node, ParameterKey.COMPONENT_CROSS_COUPLING_PERIOD_ZERO_1,utc) : startDate;
            final double amplitude1 =
                    parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_AMPLITUDE_1) ? 
                    parser.getDouble(node, ParameterKey.COMPONENT_CROSS_COUPLING_AMPLITUDE_1) : 0.0;
                    
            // Default values for optional second X-coupling direction 
            double standardDeviation2           = 0.0;
            Vector3D couplingDirection2         = couplingDirection1;
            double       constantCouplingRatio2 = 0.0;
            double       secularVariation2      = 0.0;
            AbsoluteDate secularZero2           = startDate;
            double       period2                = 0.0;
            AbsoluteDate harmonicZero2          = startDate;
            double       amplitude2             = 0.0;
            
            // Optional second X-coupling direction detailed definition (when present)
            if(parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_SECULAR_VARIATION_2)){        
                standardDeviation2 =
                        parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_STANDARD_DEVIATION2) ?
                        parser.getDouble(node, ParameterKey.COMPONENT_CROSS_COUPLING_STANDARD_DEVIATION2) : 0.0;
                couplingDirection2 = 
                		parser.getVector(node, ParameterKey.COMPONENT_CROSS_COUPLING_DIRECTION_2);
                constantCouplingRatio2 =
                        parser.getDouble(node, ParameterKey.COMPONENT_CROSS_COUPLING_CONSTANT_RATIO_2);
                secularVariation2 =
                        parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_SECULAR_VARIATION_2) ? 
                        parser.getDouble(node, ParameterKey.COMPONENT_CROSS_COUPLING_SECULAR_VARIATION_2) : 0.0;
                secularZero2 =
                        parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_SECULAR_ZERO_2) ? 
                        parser.getDate(node, ParameterKey.COMPONENT_CROSS_COUPLING_SECULAR_ZERO_2,utc) : startDate;
                period2 =
                        parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_PERIOD_2) ? 
                        parser.getDouble(node, ParameterKey.COMPONENT_CROSS_COUPLING_PERIOD_2) : 0.0;
                harmonicZero2 =
                        parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_PERIOD_ZERO_2) ? 
                        parser.getDate(node, ParameterKey.COMPONENT_CROSS_COUPLING_PERIOD_ZERO_2,utc) : startDate;
                amplitude2 =
                        parser.containsKey(node, ParameterKey.COMPONENT_CROSS_COUPLING_AMPLITUDE_2) ? 
                        parser.getDouble(node, ParameterKey.COMPONENT_CROSS_COUPLING_AMPLITUDE_2) : 0.0;
            }
	                                    
            return new ManeuverCrossCoupling(getIndices(parser, node, skat), name, nominalDirection, skat.getGenerator(),
                    standardDeviation1, couplingDirection1, constantCouplingRatio1, secularVariation1, secularZero1, period1, harmonicZero1, amplitude1,
                    standardDeviation2, couplingDirection2, constantCouplingRatio2, secularVariation2, secularZero2, period2, harmonicZero2, amplitude2);
        }
    },

    /** Constant for maneuver splitting scenario component. */
    MANEUVER_ECLIPSE_CONSTRAINT() {
        /** {@inheritDoc} */
        public ScenarioComponent parse(final SkatFileParser parser, final Tree node, final Skat skat)
            throws OrekitException, SkatException {
            final String name =
                    parser.getString(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_NAME);
            
            // entry_delay field may either be double[n][2] (variable, function of mass) or double (constant, for backward compatibility) 
            double[][] entryDelay;
            if(parser.getValue(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_ENTRY_DELAY).getType()
            		== SkatParser.ARRAY){
            	entryDelay = parser.getDoubleArray2(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_ENTRY_DELAY);
            }
            else{
            	entryDelay = new double[1][2];
            	entryDelay[0][0] = parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_ENTRY_DELAY);
            	entryDelay[0][1] = 0.;
            }
            
            // exit_delay field may either be double[n][2] (variable, function of mass) or double (constant, for backward compatibility) 
            double[][] exitDelay;
            if(parser.getValue(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_EXIT_DELAY).getType()
            		== SkatParser.ARRAY){
             	exitDelay = parser.getDoubleArray2(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_EXIT_DELAY);
            }
            else{
             	exitDelay = new double[1][2];
            	exitDelay[0][0] = parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_EXIT_DELAY);
            	exitDelay[0][1] = 0.;
            }
            
            final int orbitsSeparation =
                    parser.getInt(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_ORBITS_SEPARATION);
            final double durationMinRatio =
                    parser.getDouble(node, ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_DURATION_MIN_RATIO);
            final boolean compensateLongBurnAndNodeAsymmetry = parser.getBoolean(node,   ParameterKey.COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_LONG_BURN_COMPENSATION);            
            return new ManeuverEclipseConstraint(getIndices(parser, node, skat), name,
                                                 entryDelay, exitDelay, orbitsSeparation, durationMinRatio, compensateLongBurnAndNodeAsymmetry,
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
                        parser.getEnumerate(propagatorNode, ParameterKey.COMPONENT_PROPAGATION_METHOD,
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

            // build the component
            Propagation propagation =
                    new Propagation(indices, randomizers, truncationManeuverName, truncationManeuverDelay);

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
