/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.analysis.MultivariateRealFunction;
import org.apache.commons.math.optimization.BaseMultivariateRealOptimizer;
import org.apache.commons.math.optimization.ConvergenceChecker;
import org.apache.commons.math.optimization.RealPointValuePair;
import org.apache.commons.math.optimization.direct.CMAESOptimizer;
import org.apache.commons.math.util.FastMath;

import eu.eumetsat.skat.Skat;
import eu.eumetsat.skat.control.BoundedNelderMead;
import eu.eumetsat.skat.control.SKParameter;
import eu.eumetsat.skat.strategies.TunableManeuver;

/** Enumerate for the supported optimizers.
 */
public enum SupportedOptimizer {

    /** Constant for Nelder-Mead. */
    NELDER_MEAD() {
        /** {@inheritDoc} */
        public BaseMultivariateRealOptimizer<MultivariateRealFunction>
            parse(final SkatFileParser parser, final Tree node,
                  final TunableManeuver[] maneuvers, final double stopCriterion, final Skat skat) {
            return new BoundedNelderMead(parser.getDouble(node, ParameterKey.NELDER_MEAD_INITIAL_SIMPLEX_SIZE_RATIO),
                                         new Checker(maneuvers, stopCriterion));
        }
    },

    /** Constant for Covariance Matrix Adaptation Evolution Strategy (CMA-ES). */
    CMA_ES() {
        /** {@inheritDoc} */
        public BaseMultivariateRealOptimizer<MultivariateRealFunction>
            parse(final SkatFileParser parser, final Tree node,
                  final TunableManeuver[] maneuvers, final double stopCriterion, final Skat skat) {
            double[][] boundaries = getBoundaries(maneuvers);
            final double[] inputSigma        = new double[boundaries[0].length];
            for (int i = 0; i < inputSigma.length; ++i) {
                inputSigma[i] = (boundaries[1][i] - boundaries[0][i]) / 3.0;
            }
            return new CMAESOptimizer(parser.getInt(node, ParameterKey.CMAES_POPULATION_SIZE),
                                      inputSigma, boundaries,
                                      parser.getInt(node, ParameterKey.CMAES_MAX_ITERATIONS),
                                      parser.getDouble(node, ParameterKey.CMAES_STOP_FITNESS),
                                      true, 0, 0, skat.getGenerator(), true,
                                      new Checker(maneuvers, stopCriterion));
        }
    };

    /** Parse an input data tree to build a scenario component.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param maneuvers maneuvers to optimize
     * @param stopCriterion stop criterion on global value
     * @param skat enclosing Skat tool
     * @return parsed component
     */
    public abstract BaseMultivariateRealOptimizer<MultivariateRealFunction>
        parse(SkatFileParser parser, Tree node, TunableManeuver[] maneuvers, double stopCriterion, Skat skat);

    /** Get the parameters boundaries.
     * @param maneuvers maneuvers to optimize
     * @return maneuvers parameters boundaries
     */
    private static double[][] getBoundaries(final TunableManeuver[] maneuvers) {

        int nbParameters = 0;
        for (int i = 0; i < maneuvers.length; ++i) {
            for (final SKParameter parameter : maneuvers[i].getParameters()) {
                if (parameter.isTunable()) {
                    ++nbParameters;
                }
            }
        }

        final double[][] boundaries = new double[2][nbParameters];

        int index = 0;
        for (int i = 0; i < maneuvers.length; ++i) {
            for (final SKParameter parameter : maneuvers[i].getParameters()) {
                if (parameter.isTunable()) {
                    boundaries[0][index] = parameter.getMin();
                    boundaries[1][index] = parameter.getMax();
                    ++index;
                }
            }
        }

        return boundaries;

    }

    /** Local class for convergence checking. */
    private static class Checker implements ConvergenceChecker<RealPointValuePair> {

        /** Maneuvers. */
        private final TunableManeuver[] maneuvers;

        /** Stop criterion on global value. */
        private final double stopCriterion;

        /** Simple constructor.
         * @param maneuvers maneuvers to optimize
         * @param stopCriterion stop criterion on global value
         */
        public Checker(final TunableManeuver[] maneuvers, final double stopCriterion) {
            this.maneuvers     = maneuvers.clone();
            this.stopCriterion = stopCriterion;
        }

        /** {@inheritDoc} */
        public boolean converged(final int iteration, final RealPointValuePair previous,
                                 final RealPointValuePair current) {

            // first check directly the criterion on the function value
            if (current.getValue() <= stopCriterion) {
                return true;
            }

            // get the optimal parameters values on the last iterations
            final double[] p = previous.getPoint();
            final double[] c = current.getPoint();

            // check the evolution of each parameter
            int index = 0;
            for (final TunableManeuver maneuver : maneuvers) {
                for (final SKParameter parameter : maneuver.getParameters()) {
                    if (parameter.isTunable()) {
                        if (FastMath.abs(c[index] - p[index]) > parameter.getConvergence()) {
                            return false;
                        }
                        ++index;
                    }
                }
            }

            // all parameters are within their convergence threshold
            return true;

        }
        
    }

}
