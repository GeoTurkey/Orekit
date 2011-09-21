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
     * @param monitorable object containing the time-dependent value
     */
    void startMonitoring(Monitorable monitorable);

    /** Monitor a time-dependent value.
     * <p>
     * This method is called each time a monitored value changes. As soon as
     * it has been called once for the first value that changes, the
     * {@link #startMonitoring(Monitorable) startMonitoring} should not be
     * called anymore, so implementation may ignore spurious out-of-sync
     * calls to preserve their internal consistency (for example number
     * of columns in CSV files).
     * </p>
     * @param monitorable object containing the time-dependent value
     */
    void valueChanged(Monitorable monitorable);

    /** Notifies a monitor that monitoring should stop for all values.
     * <p>
     * This method is called once at simulation end.
     * </p>
     */
    void stopMonitoring();

}
