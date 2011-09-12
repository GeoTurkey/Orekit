/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.List;

/**
 * Interface for objects that provides {@link ControlParameter control parameters}.
 * <p>
 * Station-keeping control parameters are the tuning parameters
 * used at each station-keeping cycle to get the associated
 * {@link StationKeepingGoal station-keeping goal} as close to
 * its {@link StationKeepingGoal#getTarget() target} as possible.
 * They correspond typically to maneuver amplitudes or dates.
 * </p>
 * @see StationKeepingGoal
 * @see ControlParameter
 * @author Luc Maisonobe
 */
public interface ControlParametersSet {

    /** Get the control parameters.
     * @return control parameters
     */
    List<ControlParameter> getParameters();

}
