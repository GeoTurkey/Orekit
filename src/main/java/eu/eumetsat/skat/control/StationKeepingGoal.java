/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;


/**
 * Interface representing a station-keeping goal that should be achieved.
 * <p>
 * Station-keeping goals can be for example to ensure that spacecraft
 * remains within its allowed slot with as large margins as possible.
 * </p>
 * @see ControlParametersSet
 * @author Luc Maisonobe
 */
public interface StationKeepingGoal {

    /** Get the name of the goal.
     * @return name of the goal
     */
    String getName();

    /** Get the target vector of the goal.
     * @return target vector of the goal
     */
    double getTarget();

    /** Get the achieved value of the goal.
     * <p>
     * The achieved value will change at the end
     * of each attempted simulation, depending
     * on the control parameters. The control
     * parameters will be set up by the station-keeping
     * engine in order to reach a value as close to the
     * {@link #getTarget target} as possible.
     * </p>
     * @return achieved value of the goal
     */
    double getAchievedValue();

}
