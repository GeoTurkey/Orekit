/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.ode.nonstiff.AdaptiveStepsizeIntegrator;
import org.apache.commons.math.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math.util.FastMath;
import org.orekit.attitudes.LofOffset;
import org.orekit.errors.OrekitException;
import org.orekit.forces.ForceModel;
import org.orekit.forces.SphericalSpacecraft;
import org.orekit.forces.drag.DragForce;
import org.orekit.forces.drag.HarrisPriester;
import org.orekit.forces.gravity.CunninghamAttractionModel;
import org.orekit.forces.gravity.ThirdBodyAttraction;
import org.orekit.forces.gravity.potential.PotentialCoefficientsProvider;
import org.orekit.forces.radiation.SolarRadiationPressure;
import org.orekit.frames.LOFType;
import org.orekit.orbits.Orbit;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.numerical.NumericalPropagator;

import eu.eumetsat.skat.Skat;

/** Enumerate for the supported propagators.
 */
public enum SupportedPropagator {

    /** Constant for numerical propagator. */
    NUMERICAL() {
        /** {@inheritDoc} */
        public Propagator parse(final SkatFileParser parser, final Tree node,
                                final Skat skat, final int spacecraftIndex)
            throws OrekitException, SkatException {

            final Orbit initialOrbit = skat.getInitialOrbit(spacecraftIndex);
            final double minStep = parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_MIN_STEP);
            final double maxStep = parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_MAX_STEP);
            final double dP      = parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_TOLERANCE);
            final double[][] tolerance = NumericalPropagator.tolerances(dP, initialOrbit, initialOrbit.getType());
            final AdaptiveStepsizeIntegrator integrator =
                    new DormandPrince853Integrator(minStep, maxStep, tolerance[0], tolerance[1]);
            integrator.setInitialStepSize(FastMath.sqrt(minStep * maxStep));
            final NumericalPropagator numPropagator = new NumericalPropagator(integrator);
            numPropagator.setAttitudeProvider(new LofOffset(initialOrbit.getFrame(), LOFType.TNW));

            // Earth gravity field
            PotentialCoefficientsProvider gravityField = skat.getgravityField();
            final int degree = parser.getInt(node, ParameterKey.NUMERICAL_PROPAGATOR_GRAVITY_FIELD_DEGREE);
            final int order  = parser.getInt(node, ParameterKey.NUMERICAL_PROPAGATOR_GRAVITY_FIELD_ORDER);
            ForceModel gravity = new CunninghamAttractionModel(skat.getEarth().getBodyFrame(),
                                                               gravityField.getAe(),
                                                               gravityField.getMu(),
                                                               gravityField.getC(degree, order, false),
                                                               gravityField.getS(degree, order, false));
            numPropagator.addForceModel(gravity);

            double dragCoeff =
                parser.containsKey(node, ParameterKey.NUMERICAL_PROPAGATOR_DRAG_COEFFICIENT) ?
                parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_DRAG_COEFFICIENT) : 0.0;
            double absorptionCoeff =
                parser.containsKey(node, ParameterKey.NUMERICAL_PROPAGATOR_ABSORPTION_COEFFICIENT) ?
                parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_ABSORPTION_COEFFICIENT) : 0.0;
            double reflectionCoeff =
                parser.containsKey(node, ParameterKey.NUMERICAL_PROPAGATOR_REFLECTION_COEFFICIENT) ?
                parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_REFLECTION_COEFFICIENT) : 0.0;
            double crossSection =
                parser.containsKey(node, ParameterKey.NUMERICAL_PROPAGATOR_CROSS_SECTION) ?
                parser.getDouble(node, ParameterKey.NUMERICAL_PROPAGATOR_CROSS_SECTION) : 0.0;

            // third bodies
            final Tree thirdBodiesNode = parser.getValue(node, ParameterKey.NUMERICAL_PROPAGATOR_THIRD_BODIES);
            for (int i = 0; i < parser.getElementsNumber(thirdBodiesNode); ++i) {
                final String body = parser.getIdentifier(thirdBodiesNode, i);
                if (body.equals("SUN")) {
                    numPropagator.addForceModel(new ThirdBodyAttraction(skat.getSun()));
                } else if (body.equals("MOON")) {
                    numPropagator.addForceModel(new ThirdBodyAttraction(skat.getMoon()));
                } else {
                    throw new SkatException(SkatMessages.UNSUPPORTED_KEY, body, thirdBodiesNode.getLine(),
                                            parser.getInputName(), "SUN, MOON");
                }
            }

            // radiation pressure
            SphericalSpacecraft s =
                new SphericalSpacecraft(crossSection, dragCoeff, absorptionCoeff, reflectionCoeff);

            if ((absorptionCoeff > 0) || (reflectionCoeff > 0)) {
                numPropagator.addForceModel(new SolarRadiationPressure(skat.getSun(),
                                                                       skat.getEarth().getEquatorialRadius(),
                                                                       s));
            }

            // drag
            if (dragCoeff > 0) {
                numPropagator.addForceModel(new DragForce(new HarrisPriester(skat.getSun(), skat.getEarth()),
                                                          s));
            }

            numPropagator.setOrbitType(initialOrbit.getType());
            return numPropagator;

        }
    },

    /** Constant for Draper Semi-Analytical Satellite Theory (DSST). */
    DSST() {
        /** {@inheritDoc} */
        public Propagator parse(final SkatFileParser parser, final Tree node,
                                final Skat skat, final int spacecraftIndex)
            throws OrekitException, SkatException {
            // TODO implement semi-analytical propagation
            throw SkatException.createInternalError(null);
        }
    };

    /** Parse an input data tree to build a propagator.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param skat enclosing Skat tool
     * @param spacecraftIndex index of the spacecraft to be propagated
     * @return parsed component
     * @exception OrekitException if propagator cannot be built
     * @exception if a third body name is not recognized
     */
    public abstract Propagator parse(final SkatFileParser parser, final Tree node,
                                     final Skat skat, final int spacecraftIndex)
        throws OrekitException, SkatException;

}
