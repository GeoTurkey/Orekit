/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.orekit.time.AbsoluteDate;

/** Interface representing time-dependent values from a single spacecraft
 * that can be monitored.
 * <p>
 * Monitoring time-dependent values allows storing their evolution in
 * csv files or displaying them as graphical curves.
 * </p>
 * 
 * @see MonitorMono
 * @author Luc Maisonobe
 */
public interface MonitorableMono {

    /** Register a {@link MonitorMono} interested in monitoring this value.
     * <p>
     * When a monitor is registered, its {@link
     * MonitorMono#startMonitoring(MonitorableMono) startMonitoring} method is
     * called as a side effected of registration.
     * </p>
     * @param nbSpacecrafts number of spacecrafts
     * @param monitor monitor to register
     */
    void register(int nbSpacecrafts, MonitorMono monitor);

    /** Get the name of the time-dependent value.
     * @return name of the time-dependent value
     */
    String getName();

    /** Get the current date.
     * @return current date
     */
    AbsoluteDate getDate();

    /** Get the current value for a specific spacecraft.
     * @param spacecraftIdx index of the spacecraft
     * @return current value
     */
    double[] getValue(int spacecraftIdx);

}
