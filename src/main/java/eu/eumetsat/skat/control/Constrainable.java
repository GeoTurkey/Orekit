/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;


/**
 * Interface representing a station-keeping element that can be subject to a constraint.
 * <p>
 * Constraints are limited to simple bound constraints.
 * </p>
 * @author Luc Maisonobe
 */
public interface Constrainable {

    /** Check if a constraint is enabled.
     * @return true if a constraint is enabled.
     */
    boolean isConstrained();

    /** Get the minimal allowed value for the parameter.
     * <p>
     * If no constraint is enabled (i.e. if {@link #isConstrained()}
     * returns false, this method should return {@code Double.NEGATIVE_INFINITY}.
     * </p>
     * @return minimal allowed value
     */
    double getMin();

    /** Get the maximal allowed value for the parameter.
     * <p>
     * If no constraint is enabled (i.e. if {@link #isConstrained()}
     * returns false, this method should return {@code Double.POSITIVE_INFINITY}.
     * </p>
     * @return maximal allowed value
     */
    double getMax();

}
