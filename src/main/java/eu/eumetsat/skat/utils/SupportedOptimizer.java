/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.analysis.MultivariateRealFunction;
import org.apache.commons.math.optimization.BaseMultivariateRealOptimizer;
import org.apache.commons.math.optimization.direct.BOBYQAOptimizer;
import org.apache.commons.math.optimization.direct.CMAESOptimizer;

/** Enumerate for the supported optimizers.
 */
public enum SupportedOptimizer {

    /** Constant for Covariance Matrix Adaptation Evolution Strategy (CMA-ES). */
    CMA_ES() {
        /** {@inheritDoc} */
        public BaseMultivariateRealOptimizer<MultivariateRealFunction>
            parse(final SkatFileParser parser, final Tree node) {
            return new CMAESOptimizer(parser.getInt(node, ParameterKey.OPTIMIZER_NB_POINTS));
        }
    },

    /** Constant for BOBYQA. */
    BOBYQA() {
        /** {@inheritDoc} */
        public BaseMultivariateRealOptimizer<MultivariateRealFunction>
            parse(final SkatFileParser parser, final Tree node) {
            return new BOBYQAOptimizer(parser.getInt(node, ParameterKey.OPTIMIZER_NB_POINTS));
        }
    };

    /** Parse an input data tree to build a scenario component.
     * @param parser input file parser
     * @param node data node containing component configuration parameters
     * @return parsed component
     */
    public abstract BaseMultivariateRealOptimizer<MultivariateRealFunction>
        parse(final SkatFileParser parser, final Tree node);

}
