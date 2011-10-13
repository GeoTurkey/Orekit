/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import org.orekit.time.AbsoluteDate;

/** Interface representing time-dependent values from two spacecrafts
 * that can be monitored.
 * <p>
 * Monitoring time-dependent values allows storing their evolution in
 * csv files or displaying them as graphical curves.
 * </p>
 * 
 * @see MonitorDuo
 * @author Luc Maisonobe
 */
public interface MonitorableDuo {

    /** Register a {@link MonitorDuo} interested in monitoring this value.
     * <p>
     * When a monitor is registered, its {@link
     * MonitorDuo#startMonitoring(MonitorableDuo) startMonitoring} method is
     * called as a side effected of registration.
     * </p>
     * @param nbSpacecrafts number of spacecrafts
     * @param monitor monitor to register
     */
    void register(int nbSpacecrafts, MonitorDuo monitor);

    /** Get the name of the time-dependent value.
     * @return name of the time-dependent value
     */
    String getName();

    /** Get the current date.
     * @return current date
     */
    AbsoluteDate getDate();

    /** Get the current value.
     * @param spacecraftIdx1 index of the first spacecraft
     * @param spacecraftIdx2 index of the second spacecraft
     * @return current value
     */
    double[] getValue(int spacecraftIdx1, int spacecraftIdx2);

}
