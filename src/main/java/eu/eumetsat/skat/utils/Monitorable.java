/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.orekit.time.AbsoluteDate;

/** Interface representing time-dependent values that can be monitored.
 * <p>
 * Monitoring time-dependent values allows storing their evolution in
 * csv files or displaying them as graphical curves.
 * </p>
 * 
 * @see Monitor
 * @author Luc Maisonobe
 */
public interface Monitorable {

    /** Register a {@link Monitor} interested in monitoring this value.
     * <p>
     * When a monitor is registered, its {@link
     * Monitor#startMonitoring(Monitorable) startMonitoring} method is
     * called as a side effected of registration.
     * </p>
     * @param monitor monitor to register
     */
    void register(Monitor monitor);

    /** Get the name of the time-dependent value.
     * @return name of the time-dependent value
     */
    String getName();

    /** Get the current date.
     * @return current date
     */
    AbsoluteDate getDate();

    /** Get the current value.
     * @return current value
     */
    double[] getValue();

}
