/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

/**
 * Interface representing a station-keeping control parameter.
 * <p>
 * Station-keeping control parameters are the tuning parameters
 * used at each station-keeping cycle to get the associated
 * {@link StationKeepingGoal station-keeping goal} as close to
 * its {@link StationKeepingGoal#getTarget() target} as possible.
 * They correspond typically to maneuver amplitudes or dates.
 * </p>
 * @author Luc Maisonobe
 */
public interface ControlParameter {

    /** Get the name of the parameter.
     * @return name of the parameter
     */
    String getName();

    /** Get the minimal allowed value for the parameter.
     * @return minimal allowed value
     */
    double getMin();

    /** Get the maximal allowed value for the parameter.
     * @return maximal allowed value
     */
    double getMax();

    /** Get the current value of the parameter.
     * @return current value of the parameter
     */
    double getValue();

    /** Set the current value of the parameter.
     * @parem value current value of the parameter
     */
    void setValue(double value);

}
