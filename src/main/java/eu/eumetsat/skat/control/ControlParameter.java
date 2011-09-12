/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

/**
 * Base class representing a station-keeping control parameter.
 * <p>
 * Station-keeping control parameters are the tuning parameters
 * used at each station-keeping cycle to get the associated
 * {@link StationKeepingGoal station-keeping goal} as close to
 * its {@link StationKeepingGoal#getTarget() target} as possible.
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
public abstract class ControlParameter {

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

    /** Simple constructor.
     * @param name name of the parameter
     * @param min minimal allowed value for the parameter
     * @param max maximal allowed value for the parameter
     * @param value current value of the parameter
     * @param tunable tunable flag
     */
    protected ControlParameter(final String name,
                               final double min, final double max,
                               final double value, final boolean tunable) {
        this.name    = name;
        this.min     = min;
        this.max     = max;
        setValue(value);
        setTunable(tunable);
    }

    /** Get the name of the parameter.
     * @return name of the parameter
     */
    public String getName() {
        return name;
    }

    /** Get the minimal allowed value for the parameter.
     * @return minimal allowed value
     */
    public double getMin() {
        return min;
    }

    /** Get the maximal allowed value for the parameter.
     * @return maximal allowed value
     */
    public double getMax() {
        return max;
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
    public void setValue(double value) {
        this.value = value;
        valueChanged();
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
