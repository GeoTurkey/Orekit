/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.util.ArrayList;
import java.util.List;

import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.ode.nonstiff.AdaptiveStepsizeIntegrator;
import org.apache.commons.math.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math.util.FastMath;
import org.orekit.attitudes.LofOffset;
import org.orekit.errors.OrekitException;
import org.orekit.forces.ForceModel;
import org.orekit.forces.SphericalSpacecraft;
import org.orekit.forces.drag.DragForce;
import org.orekit.forces.gravity.CunninghamAttractionModel;
import org.orekit.forces.gravity.ThirdBodyAttraction;
import org.orekit.forces.gravity.potential.PotentialCoefficientsProvider;
import org.orekit.forces.radiation.SolarRadiationPressure;
import org.orekit.frames.LOFType;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.propagation.semianalytical.dsst.DSSTPropagator;
import org.orekit.propagation.semianalytical.dsst.dsstforcemodel.DSSTAtmosphericDrag;
import org.orekit.propagation.semianalytical.dsst.dsstforcemodel.DSSTCentralBody;
import org.orekit.propagation.semianalytical.dsst.dsstforcemodel.DSSTForceModel;
import org.orekit.propagation.semianalytical.dsst.dsstforcemodel.DSSTSolarRadiationPressure;
import org.orekit.propagation.semianalytical.dsst.dsstforcemodel.DSSTThirdBody;
import org.orekit.utils.Constants;

import eu.eumetsat.skat.Skat;

/** Enumerate for the supported propagators.
 */
public enum SupportedPropagator {

    /** Constant for numerical propagator. */
    NUMERICAL() {
        /** {@inheritDoc} */
        public PropagatorRandomizer parse(final SkatFileParser parser, final Tree node,
                                          final Skat skat, final int spacecraftIndex)
            throws OrekitException, SkatException {

            final List<ForceModel> forceModels = new ArrayList<ForceModel>();
            final double dP      = parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_TOLERANCE);
            final double minStep = parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_MIN_STEP);
            final double maxStep = parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_MAX_STEP);

            // Earth gravity field
            PotentialCoefficientsProvider gravityField = skat.getgravityField();
            final int degree = parser.getInt(node, ParameterKey.NUMERICAL_PROPAGATOR_GRAVITY_FIELD_DEGREE);
            final int order  = parser.getInt(node, ParameterKey.NUMERICAL_PROPAGATOR_GRAVITY_FIELD_ORDER);
            final ForceModel gravity = new CunninghamAttractionModel(skat.getEarth().getBodyFrame(),
                                                                     gravityField.getAe(),
                                                                     gravityField.getMu(),
                                                                     gravityField.getC(degree, order, false),
                                                                     gravityField.getS(degree, order, false));
            forceModels.add(gravity);

            // drag
            final double dragStandardDeviation =
                    parser.containsKey(node, ParameterKey.NUMERICAL_PROPAGATOR_DRAG_STANDARD_DEVIATION) ?
                    parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_DRAG_STANDARD_DEVIATION) : 0.0;
            final double dragCrossSection =
                    parser.containsKey(node, ParameterKey.NUMERICAL_PROPAGATOR_DRAG_CROSS_SECTION) ?
                    parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_DRAG_CROSS_SECTION) : 0.0;
            final double dragCoeff =
                parser.containsKey(node, ParameterKey.NUMERICAL_PROPAGATOR_DRAG_COEFFICIENT) ?
                parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_DRAG_COEFFICIENT) : 0.0;

            // solar radiation pressure
            final double srpStandardDeviation =
                    parser.containsKey(node, ParameterKey.NUMERICAL_PROPAGATOR_SRP_STANDARD_DEVIATION) ?
                    parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_SRP_STANDARD_DEVIATION) : 0.0;
            final double srpCrossSection =
                parser.containsKey(node, ParameterKey.NUMERICAL_PROPAGATOR_SRP_CROSS_SECTION) ?
                parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_SRP_CROSS_SECTION) : 0.0;
            final double absorptionCoeff =
                parser.containsKey(node, ParameterKey.NUMERICAL_PROPAGATOR_ABSORPTION_COEFFICIENT) ?
                parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_ABSORPTION_COEFFICIENT) : 0.0;
            final double reflectionCoeff =
                parser.containsKey(node, ParameterKey.NUMERICAL_PROPAGATOR_REFLECTION_COEFFICIENT) ?
                parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_REFLECTION_COEFFICIENT) : 0.0;

            // third bodies
            if (parser.containsKey(node, ParameterKey.NUMERICAL_PROPAGATOR_THIRD_BODIES)){
                final Tree thirdBodiesNode = parser.getValue(node, ParameterKey.NUMERICAL_PROPAGATOR_THIRD_BODIES);
                for (int i = 0; i < parser.getElementsNumber(thirdBodiesNode); ++i) {
                    switch ((ThirdBody) parser.getEnumerate(thirdBodiesNode, i, ThirdBody.class)) {
                    case SUN :
                        forceModels.add(new ThirdBodyAttraction(skat.getSun()));
                        break;
                    case MOON :
                        forceModels.add(new ThirdBodyAttraction(skat.getMoon()));
                        break;
                    default :
                        // this should never happen
                        throw SkatException.createInternalError(null);
                    }
                }
            }
            

            return new PropagatorRandomizer() {
                
                /** {@inheritDoc} */
                public Propagator getPropagator(final SpacecraftState initialState) throws OrekitException {

                    // create integrator
                    final double[][] tolerance = NumericalPropagator.tolerances(dP, initialState.getOrbit(),
                                                                                initialState.getOrbit().getType());
                    final AdaptiveStepsizeIntegrator integrator =
                            new DormandPrince853Integrator(minStep, maxStep, tolerance[0], tolerance[1]);
                    integrator.setInitialStepSize(FastMath.sqrt(minStep * maxStep));

                    // create propagator
                    final NumericalPropagator numPropagator = new NumericalPropagator(integrator);
                    numPropagator.resetInitialState(initialState);

                    // general settings
                    numPropagator.setAttitudeProvider(new LofOffset(initialState.getFrame(), LOFType.VNC));
                    numPropagator.setOrbitType(initialState.getOrbit().getType());

                    // add the fixed force models
                    for (final ForceModel forceModel : forceModels) {
                        numPropagator.addForceModel(forceModel);
                    }

                    // change the coefficients of the randomized force models
                    if (dragCrossSection > 0 && dragCoeff > 0) {
                        final double random = 1.0 + dragStandardDeviation * skat.getGenerator().nextGaussian();
                        final double randomizedCrossSection = dragCrossSection * FastMath.max(0, random);
                        SphericalSpacecraft s = new SphericalSpacecraft(randomizedCrossSection, dragCoeff, 0., 0.);
                        numPropagator.addForceModel(new DragForce(skat.getAtmosphere(), s));
                    }

                    if (srpCrossSection > 0 && (absorptionCoeff > 0) || (reflectionCoeff > 0)) {
                        final double random = 1.0 + srpStandardDeviation * skat.getGenerator().nextGaussian();
                        final double randomizedCrossSection = srpCrossSection * FastMath.max(0, random);
                        SphericalSpacecraft s = new SphericalSpacecraft(randomizedCrossSection, 0., absorptionCoeff, reflectionCoeff);
                        numPropagator.addForceModel(new SolarRadiationPressure(skat.getSun(),
                                                                               skat.getEarth().getEquatorialRadius(),
                                                                               s));
                    }

                    return numPropagator;

                }

            };

        }
    },

    /** Constant for Draper Semi-Analytical Satellite Theory (DSST). */
    DSST() {
        /** {@inheritDoc} */
        public PropagatorRandomizer parse(final SkatFileParser parser, final Tree node,
                                          final Skat skat, final int spacecraftIndex)
            throws OrekitException, SkatException {

            final List<DSSTForceModel> forceModels = new ArrayList<DSSTForceModel>();
            final double dP = parser.getDouble(node, ParameterKey.DSST_PROPAGATOR_TOLERANCE);

            // Earth gravity field
            PotentialCoefficientsProvider gravityField = skat.getgravityField();
            final int degree = parser.getInt(node, ParameterKey.DSST_PROPAGATOR_GRAVITY_FIELD_DEGREE);
            final int order  = parser.getInt(node, ParameterKey.DSST_PROPAGATOR_GRAVITY_FIELD_ORDER);
            forceModels.add(new DSSTCentralBody(Constants.WGS84_EARTH_ANGULAR_VELOCITY,
                                                gravityField.getAe(),
                                                gravityField.getMu(),
                                                gravityField.getC(degree, order, false),
                                                gravityField.getS(degree, order, false),
                                                null));

            // drag
            final double dragStandardDeviation =
                    parser.containsKey(node, ParameterKey.DSST_PROPAGATOR_DRAG_STANDARD_DEVIATION) ?
                    parser.getDouble(node, ParameterKey.DSST_PROPAGATOR_DRAG_STANDARD_DEVIATION) : 0.0;
            final double dragCrossSection =
                parser.containsKey(node, ParameterKey.DSST_PROPAGATOR_DRAG_CROSS_SECTION) ?
                parser.getDouble(node, ParameterKey.DSST_PROPAGATOR_DRAG_CROSS_SECTION) : 0.0;
            final double dragCoeff =
                parser.containsKey(node, ParameterKey.DSST_PROPAGATOR_DRAG_COEFFICIENT) ?
                parser.getDouble(node, ParameterKey.DSST_PROPAGATOR_DRAG_COEFFICIENT) : 0.0;

            // radiation pressure
            final double srpStandardDeviation =
                        parser.containsKey(node, ParameterKey.DSST_PROPAGATOR_SRP_STANDARD_DEVIATION) ?
                        parser.getDouble(node, ParameterKey.DSST_PROPAGATOR_SRP_STANDARD_DEVIATION) : 0.0;
            final double srpCrossSection =
                parser.containsKey(node, ParameterKey.DSST_PROPAGATOR_SRP_CROSS_SECTION) ?
                parser.getDouble(node, ParameterKey.DSST_PROPAGATOR_SRP_CROSS_SECTION) : 0.0;
            // Up to now the absorption coefficient is not used in the DSST SPR model
//          final double absorptionCoeff =
//                parser.containsKey(node, ParameterKey.DSST_PROPAGATOR_ABSORPTION_COEFFICIENT) ?
//                parser.getDouble(node, ParameterKey.DSST_PROPAGATOR_ABSORPTION_COEFFICIENT) : 0.0;
            final double reflectionCoeff =
                parser.containsKey(node, ParameterKey.DSST_PROPAGATOR_REFLECTION_COEFFICIENT) ?
                parser.getDouble(node, ParameterKey.DSST_PROPAGATOR_REFLECTION_COEFFICIENT) : 0.0;

            // third bodies
            final Tree thirdBodiesNode = parser.getValue(node, ParameterKey.DSST_PROPAGATOR_THIRD_BODIES);
            for (int i = 0; i < parser.getElementsNumber(thirdBodiesNode); ++i) {
                switch ((ThirdBody) parser.getEnumerate(thirdBodiesNode, i, ThirdBody.class)) {
                case SUN :
                    forceModels.add(new DSSTThirdBody(skat.getSun()));
                    break;
                case MOON :
                    forceModels.add(new DSSTThirdBody(skat.getMoon()));
                    break;
                default :
                    // this should never happen
                    throw SkatException.createInternalError(null);
                }
            }

            return new PropagatorRandomizer() {
                
                /** {@inheritDoc} */
                public Propagator getPropagator(final SpacecraftState initialState) throws OrekitException {

                    // create integrator
                    final double period  = initialState.getKeplerianPeriod();
                    final double minStep = period / 10.;
                    final double maxStep = period * 10.;
                    final double[][] tolerance = DSSTPropagator.tolerances(dP, initialState.getOrbit());
                    final AdaptiveStepsizeIntegrator integrator =
                            new DormandPrince853Integrator(minStep, maxStep, tolerance[0], tolerance[1]);
                    integrator.setInitialStepSize(FastMath.sqrt(minStep * maxStep));

                    // create propagator
                    final DSSTPropagator dsstPropagator =
                            new DSSTPropagator(integrator, initialState.getOrbit(),
                                               new LofOffset(initialState.getFrame(), LOFType.VNC));

                    // add the fixed force models
                    for (final DSSTForceModel forceModel : forceModels) {
                        dsstPropagator.addForceModel(forceModel);
                    }

                    // change the coefficients of the randomized force models
                    if (dragCrossSection > 0 && dragCoeff > 0) {
                        final double random = 1.0 + dragStandardDeviation * skat.getGenerator().nextGaussian();
                        final double randomizedCrossSection = dragCrossSection * FastMath.max(0, random);
                        DSSTForceModel drag = new DSSTAtmosphericDrag(skat.getAtmosphere(), dragCoeff, randomizedCrossSection);
                        dsstPropagator.addForceModel(drag);
                    }

                    if (srpCrossSection > 0 && reflectionCoeff > 0) {
                        final double random = 1.0 + srpStandardDeviation * skat.getGenerator().nextGaussian();
                        final double randomizedCrossSection = srpCrossSection * FastMath.max(0, random);
                        DSSTForceModel srp = new DSSTSolarRadiationPressure(reflectionCoeff, randomizedCrossSection,
                                                                            skat.getSun(),
                                                                            skat.getEarth().getEquatorialRadius());
                        dsstPropagator.addForceModel(srp);
                    }

                    return dsstPropagator;

                }
            };

        }
    };

    /** Parse an input data tree to build a propagator randomizer.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param skat enclosing Skat tool
     * @param spacecraftIndex index of the spacecraft to be propagated
     * @return parsed component
     * @exception OrekitException if propagator cannot be built
     * @exception if a third body name is not recognized
     */
    public abstract PropagatorRandomizer parse(final SkatFileParser parser, final Tree node,
                                               final Skat skat, final int spacecraftIndex)
        throws OrekitException, SkatException;

    /** Inner enumerate for third bodies. */
    private static enum ThirdBody {
        SUN,
        MOON;
    }

    /** Interface for generating propagators with randomized force models. */
    public static interface PropagatorRandomizer {

        /** Get a randomized propagator.
         * @param initialState initial state to set up
         * @exception OrekitException if propagator cannot be generated
         */
        Propagator getPropagator(SpacecraftState initialState) throws OrekitException ;

    }

}
