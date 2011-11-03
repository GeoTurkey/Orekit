/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.analysis.MultivariateRealFunction;
import org.apache.commons.math.optimization.BaseMultivariateRealOptimizer;
import org.apache.commons.math.optimization.direct.BOBYQAOptimizer;
import org.apache.commons.math.optimization.direct.CMAESOptimizer;

import eu.eumetsat.skat.Skat;

/** Enumerate for the supported optimizers.
 */
public enum SupportedOptimizer {

    /** Constant for Covariance Matrix Adaptation Evolution Strategy (CMA-ES). */
    CMA_ES() {
        /** {@inheritDoc} */
        public BaseMultivariateRealOptimizer<MultivariateRealFunction>
            parse(final SkatFileParser parser, final Tree node,
                  final double[][] boundaries, final Skat skat) {
            final double[] inputSigma = new double[boundaries[0].length];
            for (int i = 0; i < inputSigma.length; ++i) {
                inputSigma[i] = 0.3;
            }
            return new CMAESOptimizer(parser.getInt(node, ParameterKey.CMAES_POPULATION_SIZE),
                                      inputSigma, boundaries,
                                      parser.getInt(node, ParameterKey.CMAES_MAX_ITERATIONS),
                                      parser.getDouble(node, ParameterKey.CMAES_STOP_FITNESS),
                                      true, 0, 0, skat.getGenerator(), false);
        }
    },

    /** Constant for BOBYQA. */
    BOBYQA() {
        /** {@inheritDoc} */
        public BaseMultivariateRealOptimizer<MultivariateRealFunction>
            parse(final SkatFileParser parser, final Tree node,
                  final double[][] boundaries, final Skat skat) {
            return new BOBYQAOptimizer(parser.getInt(node, ParameterKey.BOBYQA_INTERPOLATION_POINTS));
        }
    };

    /** Parse an input data tree to build a scenario component.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @param boundaries parameters boundaries
     * @param skat enclosing Skat tool
     * @return parsed component
     */
    public abstract BaseMultivariateRealOptimizer<MultivariateRealFunction>
        parse(final SkatFileParser parser, final Tree node, final double[][] boundaries, final Skat skat);

}
