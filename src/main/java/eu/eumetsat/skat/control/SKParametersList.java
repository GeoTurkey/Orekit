/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.List;

/**
 * Interface for objects that provides {@link SKParameter control parameters}.
 * <p>
 * Station-keeping control parameters are the tuning parameters
 * used at each station-keeping cycle to get the associated
 * {@link SKControl station-keeping control} as close to
 * its {@link SKControl#getTargetValue() target} as possible.
 * They correspond typically to maneuver amplitudes or dates.
 * </p>
 * @see SKControl
 * @see SKParameter
 * @author Luc Maisonobe
 */
public interface SKParametersList {

    /** Get the control parameters.
     * @return control parameters
     */
    List<SKParameter> getParameters();

}
