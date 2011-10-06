/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.stat.descriptive.rank.Median;

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
public abstract class SKParameter implements SKElement {

    /** Name of the parameter. */
    private final String name;

    /** Minimal allowed value for the parameter. */
    private final double min;

    /** Maximal allowed value for the parameter. */
    private final double max;

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
     * @param value current value of the parameter
     * @param tunable tunable flag
     */
    protected SKParameter(final String name,
                          final double min, final double max,
                          final double value, final boolean tunable) {

        this.name    = name;
        this.min     = min;
        this.max     = max;

        // initialize the guessing base for min and max only
        guessingBase = new ArrayList<Double>();
        guessingBase.add(min);
        guessingBase.add(max);

        setValue(value);
        setTunable(tunable);

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
        valueChanged();
    }

    /** Store the last optimum value found for the parameter.
     * <p>
     * This method is used to build up samples of optimal values,
     * so the {@link #guessOptimalValue()} method can use them to
     * provide better guesses for next optimization, based on previous
     * results.
     * </p>
     * @see #guessOptimalValue()
     */
    public void storeLastOptimalValue(final double optimum) {
        guessingBase.add(optimum);
    }

    /** Get a guess for the optimal value of the parameter.
     * <p>
     * The guess is based first on min/max settings and
     * </p>
     * @return guessed optimal value of the parameter
     * @see #storeLastOptimalValue(double)
     */
    public double guessOptimalValue() {
        final Median median = new Median();
        final double[] data = new double[guessingBase.size()];
        for (int i = 0; i < data.length; ++i) {
            data[i] = guessingBase.get(i);
        }
        median.setData(data);
        return median.evaluate();
    }

    /** Notify the implementation class that the parameter value has changed.
     */
    protected abstract void valueChanged();

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

    /** Set the tunable flag.
     * @param tunable if true, the parameter becomes tunable
     */
    public void setTunable(boolean tunable) {
        this.tunable = tunable;
    }

}
