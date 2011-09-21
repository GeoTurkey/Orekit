/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

/**
 * Interface for monitoring parameters evolution during station-keeping simulation.
 * <p>
 * Objects implementing this class will be notified when a {@link Monitorable monitored}
 * value changes.
 * </p>
 * @see Monitorable
 * @author Luc Maisonobe
 */
public interface Monitor {

    /** Notifies a monitor that monitoring should start for the specified value.
     * <p>
     * This method is called once for each monitored value at simulation start,
     * before any call to {@link #valueChanged(Monitorable) valueChanged}.
     * </p>
     * <p>
     * As soon as the {@link #valueChanged(Monitorable) valueChanged} method
     * has been called at least once, this method should not be called anymore.
     * If spurious out-of-sync calls are performed, an {@link IllegalStateException}
     * is thrown.
     * </p>
     * @param monitorable object containing the time-dependent value
     * @exception IllegalArgumentException if a {@link Monitorable} with the same name
     * is already monitored
     * @exception IllegalStateException if {@link #valueChanged(Monitorable)
     * valueChanged} has already been called at least once.
     */
    void startMonitoring(Monitorable monitorable)
        throws IllegalArgumentException, IllegalStateException;

    /** Monitor a time-dependent value.
     * <p>
     * This method is called each time a monitored value changes.
     * </p>
     * @param monitorable object containing the time-dependent value
     * @exception IllegalArgumentException if the {@link Monitorable}
     * is not monitored
     */
    void valueChanged(Monitorable monitorable)
        throws IllegalArgumentException;

    /** Notifies a monitor that monitoring should stop for all values.
     * <p>
     * This method is called once at simulation end.
     * </p>
     */
    void stopMonitoring();

}
