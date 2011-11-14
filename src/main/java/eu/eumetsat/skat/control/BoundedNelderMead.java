/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.Arrays;

import org.apache.commons.math.analysis.MultivariateRealFunction;
import org.apache.commons.math.optimization.ConvergenceChecker;
import org.apache.commons.math.optimization.GoalType;
import org.apache.commons.math.optimization.RealPointValuePair;
import org.apache.commons.math.optimization.direct.MultivariateRealFunctionPenaltyAdapter;
import org.apache.commons.math.optimization.direct.NelderMeadSimplex;
import org.apache.commons.math.optimization.direct.SimplexOptimizer;


/**
 * Specialization of Nelder-Mead optimizer handling boundaries the same way
 * as the other optimizers (CMA-ES and BOBYQA).
 * @author Luc Maisonobe
 */
public class BoundedNelderMead extends SimplexOptimizer {

    /** Initial simplex ratio. */
    private final double ratio;

    /**
     * Simple constructor.
     * @param initial simplex size ratio
     * @param checker Convergence checker.
     */
    public BoundedNelderMead(final double ratio,
                             final ConvergenceChecker<RealPointValuePair> checker) {
        super(checker);
        this.ratio = ratio;
    }

    /** {@inheritDoc} */
    @Override
    public RealPointValuePair optimize(final int maxEval, final MultivariateRealFunction f,
                                       final GoalType goalType, final double[] startPoint,
                                       final double[] lower, final double[] upper) {

        // set up Nelder-Mead simplex steps
        double[] steps = new double[lower.length];
        for (int i = 0; i < steps.length; ++i) {
            steps[i] = ratio * (lower[i] - upper[i]);
        }
        setSimplex(new NelderMeadSimplex(steps));

        // wrap the bounded function using a penalty adapter
        final double offset = 1.0e10;
        final double[] scale = new double[lower.length];
        Arrays.fill(scale, goalType == GoalType.MINIMIZE ? +1.0 : -1.0);
        final MultivariateRealFunctionPenaltyAdapter wrapped =
                new MultivariateRealFunctionPenaltyAdapter(f, lower, upper, offset, scale);

        // perform optimization
        return super.optimize(maxEval, wrapped, goalType, startPoint, lower, upper);

    }

}
