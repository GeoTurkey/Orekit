/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

/**
 * Interface for monitoring single spacecraft parameters evolution
 * during station-keeping simulation.
 * <p>
 * Objects implementing this class will be notified when a {@link
 * MonitorableMono monitored} value changes.
 * </p>
 * @see MonitorableMono
 * @see MonitorDuo
 * @author Luc Maisonobe
 */
public interface MonitorMono {

    /** Notifies a monitor that monitoring should start for the specified value.
     * <p>
     * This method is called once for each monitored value at simulation start,
     * before any call to {@link #valueChanged(MonitorableMono) valueChanged}.
     * </p>
     * <p>
     * As soon as the {@link #valueChanged(MonitorableMono) valueChanged} method
     * has been called at least once, this method should not be called anymore.
     * If spurious out-of-sync calls are performed, an {@link IllegalStateException}
     * is thrown.
     * </p>
     * @param monitorable object containing the time-dependent value
     * @exception IllegalArgumentException if a {@link MonitorableMono} with the same name
     * is already monitored
     * @exception IllegalStateException if {@link #valueChanged(MonitorableMono)
     * valueChanged} has already been called at least once.
     */
    void startMonitoring(MonitorableMono monitorable)
        throws IllegalArgumentException, IllegalStateException;

    /** Monitor a time-dependent value.
     * <p>
     * This method is called each time a monitored value changes.
     * </p>
     * @param monitorable object containing the time-dependent value
     * @exception IllegalArgumentException if the {@link MonitorableMono}
     * is not monitored
     */
    void valueChanged(MonitorableMono monitorable)
        throws IllegalArgumentException;

    /** Notifies a monitor that monitoring should stop for all values.
     * <p>
     * This method is called once at simulation end.
     * </p>
     */
    void stopMonitoring();

}
