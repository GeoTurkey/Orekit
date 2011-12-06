/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.analysis.MultivariateFunction;
import org.apache.commons.math.optimization.BaseMultivariateRealOptimizer;
import org.apache.commons.math.optimization.ConvergenceChecker;
import org.apache.commons.math.optimization.GoalType;
import org.apache.commons.math.optimization.RealPointValuePair;
import org.apache.commons.math.optimization.direct.CMAESOptimizer;
import org.apache.commons.math.optimization.direct.MultivariateFunctionPenaltyAdapter;
import org.apache.commons.math.optimization.direct.NelderMeadSimplex;
import org.apache.commons.math.optimization.direct.SimplexOptimizer;
import org.apache.commons.math.util.FastMath;

import eu.eumetsat.skat.Skat;
import eu.eumetsat.skat.control.SKParameter;
import eu.eumetsat.skat.strategies.TunableManeuver;

/** Enumerate for the supported optimizers.
 */
public enum SupportedOptimizer {
    

    /** Constant for Nelder-Mead. */
    NELDER_MEAD() {
        /** {@inheritDoc} */
        public BaseMultivariateRealOptimizer<MultivariateFunction>
            parse(final SkatFileParser parser, final Tree node,
                  final TunableManeuver[] maneuvers, final double stopCriterion, 
                  final Skat skat) {

            final double[][] boundaries = getBoundaries(maneuvers);
            final int convergenceSpan = parser.getInt(node, ParameterKey.NELDER_MEAD_CONVERGENCE_SPAN);

            final SimplexOptimizer optimizer = new SimplexOptimizer(new NelderMeadChecker(maneuvers, stopCriterion, convergenceSpan)) {

                /** {@inheritDoc} */
                @Override
                public RealPointValuePair optimize(final int maxEval, final MultivariateFunction f,
                                                   final GoalType goalType, final double[] startPoint) {

                    // wrap the bounded function using a penalty adapter
                    final double offset = 1.0e10;
                    final double[] scale = new double[boundaries[0].length];
                    Arrays.fill(scale, goalType == GoalType.MINIMIZE ? +1.0 : -1.0);
                    final MultivariateFunctionPenaltyAdapter wrapped =
                            new MultivariateFunctionPenaltyAdapter(f, boundaries[0], boundaries[1], offset, scale);

                    // perform optimization
                    return super.optimize(maxEval, wrapped, goalType, startPoint);

                }
            };

            // set up Nelder-Mead simplex steps
            final double   ratio = parser.getDouble(node, ParameterKey.NELDER_MEAD_INITIAL_SIMPLEX_SIZE_RATIO);
            
            final double[] steps = new double[boundaries[0].length];
            for (int i = 0; i < steps.length; ++i) {
                steps[i] = ratio * (boundaries[0][i] - boundaries[1][i]);
            }
            optimizer.setSimplex(new NelderMeadSimplex(steps));

            return optimizer;

        }
    },

    /** Constant for Covariance Matrix Adaptation Evolution Strategy (CMA-ES). */
    CMA_ES() {
        /** {@inheritDoc} */
        public BaseMultivariateRealOptimizer<MultivariateFunction>
            parse(final SkatFileParser parser, final Tree node,
                  final TunableManeuver[] maneuvers, final double stopCriterion, 
                  final Skat skat) {
            double[][] boundaries = getBoundaries(maneuvers);
            final double[] inputSigma        = new double[boundaries[0].length];
            for (int i = 0; i < inputSigma.length; ++i) {
                inputSigma[i] = (boundaries[1][i] - boundaries[0][i]) / 3.0;
            }
            return new CMAESOptimizer(parser.getInt(node, ParameterKey.CMAES_POPULATION_SIZE),
                                      inputSigma, boundaries,
                                      parser.getInt(node, ParameterKey.CMAES_MAX_ITERATIONS),
                                      0, true, 0, 0, skat.getGenerator(), true,
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
    public abstract BaseMultivariateRealOptimizer<MultivariateFunction>
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

    /** Local Nelder Mead class for convergence checking. */
    private static class NelderMeadChecker implements ConvergenceChecker<RealPointValuePair> {

        /** Maneuvers. */
        private final TunableManeuver[] maneuvers;
        
        private Map<Integer, List<Double>> list;

        /** Stop criterion on global value. */
        private final double stopCriterion;
        
        /** Dimension problem */
        private int dimension;
        
        /** Index in the Nelder Mead comparison*/
        private int counter;
        
        /** Maximum value taken by one of the simplex contained in the */
        private double maxSimplex;
        
        /** Minimum value taken by the */
        private double minSimplex;
        
        /** Did the simplex satisfy the convergence criteria */
        private boolean totalConverged;
        
        /** Did every maneuver parameters converged */
        private boolean totalParameterCheck;        

        /** 
         * Convergence span to stop the function evaluation. Convergence span is used to evaluate the function 
         * variation over a specific number of value. If values stay smaller than the stop criterion over the 
         * convergence span, then we consider that the optimization algorithm converged.
         **/
        private int convergenceSpan;


        /** Simple constructor.
         * @param maneuvers maneuvers to optimize
         * @param stopCriterion stop criterion on global value
         */
        public NelderMeadChecker(final TunableManeuver[] maneuvers, final double stopCriterion, final int convergenceSpan) {
            this.maneuvers     = maneuvers.clone();
            this.stopCriterion = stopCriterion;
            this.convergenceSpan = convergenceSpan;
            this.totalConverged = true;
            this.totalParameterCheck = true;
            this.dimension = -1;
            this.counter = -1;
            this.minSimplex = Double.POSITIVE_INFINITY;
            this.maxSimplex = Double.NEGATIVE_INFINITY;
            this.list = new LinkedHashMap<Integer, List<Double>>();
        }

        /** {@inheritDoc} */
        public boolean converged(final int iteration, 
                                 final RealPointValuePair previous,
                                 final RealPointValuePair current) {

            // Simplex dimension
            dimension = current.getPoint().length;
            // Convergence watcher
            boolean convergence = false;
            // Evolution of each parameter watcher
            boolean currentParameterCheck = true;
            // Total evolution of maneuvers parameter

            // Current simplex list of value
            List<Double> currentList;

            int index = iteration - 1;
            if (list.containsKey(index)){
                // Get the already existing list :
                currentList = list.get(index);
                // Add the current value
                currentList.add(current.getValue());
                // Update the list :
                list.put(index, currentList);
            }else {
                // Create a new list for the new simplex
                currentList = new ArrayList<Double>();
                currentList.add(current.getValue());
                list.put(index, currentList);
                // Reset parameter watcher
                totalParameterCheck = true;
            }


       
            if (list.size() >= convergenceSpan){
                // The map is filed enough to evaluate the last simplex evolution over the convergence span:
                for (int i = iteration - convergenceSpan; i < list.size(); i++){
                    List<Double> value = list.get(i);
                    maxSimplex = FastMath.max(maxSimplex, Collections.max(value));
                    minSimplex = FastMath.min(minSimplex, Collections.min(value));
                }
                convergence = (maxSimplex - minSimplex) < stopCriterion;
               
                    // get the optimal parameters values on the last iterations
                    final double[] p = previous.getPoint();
                    final double[] c = current.getPoint();
                
                    // check the evolution of each parameter
                    index = 0;
                    for (final TunableManeuver maneuver : maneuvers) {
                        for (final SKParameter parameter : maneuver.getParameters()) {
                            if (parameter.isTunable()) {
                                if (FastMath.abs(c[index] - p[index]) > parameter.getConvergence()) {
                                    currentParameterCheck = false;
                                }
                                totalParameterCheck &= currentParameterCheck;
                                ++index;
                            }
                        }
                    }                    
                counter++;
            }else {
                return false;
            }
            
            totalConverged &= (convergence && totalParameterCheck);

            // End of simplex evaluation : reset state
            if (counter == dimension){
                // Reset state :
                minSimplex = Double.POSITIVE_INFINITY;
                maxSimplex = Double.NEGATIVE_INFINITY;
                currentParameterCheck = true;
                // If a solution has been found, reset the list 
                if (totalConverged){
                    list.clear();
                }
                totalConverged = true;
                counter = -1;
                dimension = 0;
            }            
            // Get control on function evaluation convergence (parameterCheck) and on function x-axis converging (parameterCheck) 
            return (totalParameterCheck && convergence);
        }
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
        

