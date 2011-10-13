/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

/**
 * Interface for monitoring two spacecrafts parameters evolution
 * during station-keeping simulation.
 * <p>
 * Objects implementing this class will be notified when a {@link
 * MonitorableDuo monitored} value changes.
 * </p>
 * @see MonitorableDuo
 * @see MonitorMono
 * @author Luc Maisonobe
 */
public interface MonitorDuo {

    /** Notifies a monitor that monitoring should start for the specified value.
     * <p>
     * This method is called once for each monitored value at simulation start,
     * before any call to {@link #valueChanged(MonitorableDuo) valueChanged}.
     * </p>
     * <p>
     * As soon as the {@link #valueChanged(MonitorableDuo) valueChanged} method
     * has been called at least once, this method should not be called anymore.
     * If spurious out-of-sync calls are performed, an {@link IllegalStateException}
     * is thrown.
     * </p>
     * @param monitorable object containing the time-dependent value
     * @exception IllegalArgumentException if a {@link MonitorableDuo} with the same name
     * is already monitored
     * @exception IllegalStateException if {@link #valueChanged(MonitorableDuo)
     * valueChanged} has already been called at least once.
     */
    void startMonitoring(MonitorableDuo monitorable)
        throws IllegalArgumentException, IllegalStateException;

    /** MonitorMono a time-dependent value.
     * <p>
     * This method is called each time a monitored value changes.
     * </p>
     * @param monitorable object containing the time-dependent value
     * @exception IllegalArgumentException if the {@link MonitorableDuo}
     * is not monitored
     */
    void valueChanged(MonitorableDuo monitorable)
        throws IllegalArgumentException;

    /** Notifies a monitor that monitoring should stop for all values.
     * <p>
     * This method is called once at simulation end.
     * </p>
     */
    void stopMonitoring();

}
