/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class representing a station-keeping control parameter.
 * <p>
 * Station-keeping control parameters are the tuning parameters
 * used at each station-keeping cycle to get the associated
 * {@link SKControl station-keeping control} as close to
 * its {@link SKControl#getTargetValue() target} as possible.
 * They correspond typically to maneuver amplitudes or dates.
 * </p>
 * <p>
 * This abstract class implements the commons properties management
 * (name, min value, ...). The only thing left to the specific
 * implementations is the behavior at parameter setting, since the
 * new set value must be provided in an implementation-specific way
 * to the orbit propagation.
 * </p>
 * @author Luc Maisonobe
 */
public class SKParameter implements SKElement {

    /** Name of the parameter. */
    private final String name;

    /** Minimal allowed value for the parameter. */
    private final double min;

    /** Maximal allowed value for the parameter. */
    private final double max;

    /** Convergence threshold for the parameter. */
    private final double convergence;

    /** Current value of the parameter. */
    private double value;

    /** Tunable flag. */
    private boolean tunable;

    /** Sample for guessing optimal values. */
    private List<Double> guessingBase;

    /** Simple constructor.
     * @param name name of the parameter
     * @param min minimal allowed value for the parameter
     * @param max maximal allowed value for the parameter
     * @param convergence convergence threshold for the parameter
     * @param value current value of the parameter
     * @param tunable tunable flag
     */
    public SKParameter(final String name,
                       final double min, final double max, final double convergence,
                       final double value, final boolean tunable) {

        this.name        = name;
        this.min         = min;
        this.max         = max;
        this.convergence = convergence;

        // initialize the guessing base for min and max only
        guessingBase = new ArrayList<Double>();
        guessingBase.add(min);
        guessingBase.add(max);

        setValue(value);
        this.tunable = tunable;

    }

    /** Check if a constraint is enabled.
     * @return true if a constraint is enabled.
     */
    public boolean isConstrained() {
        return (getMin() == Double.NEGATIVE_INFINITY) &&
               (getMax() == Double.POSITIVE_INFINITY);
    }

    /** Get the minimal allowed value for the parameter.
     * <p>
     * If no constraint is enabled (i.e. if {@link #isConstrained()}
     * returns false, this method should return {@code Double.NEGATIVE_INFINITY}.
     * </p>
     * @return minimal allowed value
     */
    public double getMin() {
        return min;
    }

    /** Get the maximal allowed value for the parameter.
     * <p>
     * If no constraint is enabled (i.e. if {@link #isConstrained()}
     * returns false, this method should return {@code Double.POSITIVE_INFINITY}.
     * </p>
     * @return maximal allowed value
     */
    public double getMax() {
        return max;
    }

    /** Get the convergence threshold for the parameter.
     * @return convergence threshold
     */
    public double getConvergence() {
        return convergence;
    }

    /** Get the name of the parameter.
     * @return name of the parameter
     */
    public String getName() {
        return name;
    }

    /** Get the current value of the parameter.
     * @return current value of the parameter
     */
    public double getValue() {
        return value;
    }

    /** Set the current value of the parameter.
     * @parem value current value of the parameter
     */
    public void setValue(final double value) {
        this.value = value;
    }

    /** Check if the parameter can be tuned.
     * <p>
     * A tunable parameter can be automatically tuned by the
     * station-keeping {@link ControlLoop control loop}.
     * </p>
     * @return true if the parameter can be tuned,
     * false if its value is fixed
     */
    public boolean isTunable() {
        return tunable;
    }

}
