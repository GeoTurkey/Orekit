/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import org.apache.commons.math.exception.DimensionMismatchException;
import org.orekit.time.AbsoluteDate;

/** Basic implementation of the {@link Monitorable monitorable} time-dependent values.
 * @author Luc Maisonobe
 */
public class BasicMonitorable implements Monitorable {

    /** Name of the time-dependent value. */
    private final String name;

    /** Current date. */
    private AbsoluteDate date;

    /** Current value. */
    private final double[] value;

    /** Monitors interested in monitoring this value. */
    private final Set<Monitor> monitors;

    /** Simple constructor.
     * @param name name of the time-dependent value
     * @param dimension expected dimension of the value
     */
    public BasicMonitorable(final String name, final int dimension) {
        this.name  = name;
        this.date  = AbsoluteDate.PAST_INFINITY;
        this.value = new double[dimension];
        Arrays.fill(value, Double.NaN);
        this.monitors = new HashSet<Monitor>();
    }

    /** {@inheritDoc} */
    public void register(Monitor monitor) {
        monitors.add(monitor);
        monitor.startMonitoring(this);
    }

    /** {@inheritDoc} */
    public String getName() {
        return name;
    }

    /** {@inheritDoc} */
    public AbsoluteDate getDate() {
        return date;
    }

    /** {@inheritDoc} */
    public double[] getValue() {
        return value.clone();
    }

    /** Set the current date and value, and notifies all monitors.
     * @param currentDate current date
     * @param curentValue current value
     * @exception DimensionMismatchException if current value dimension
     * does not match the dimension specified at construction
     */
    public void setDateAndValue(final AbsoluteDate currentDate, final double[] currentValue)
        throws DimensionMismatchException {

        // safety check
        if (currentValue.length != value.length) {
            throw new DimensionMismatchException(currentValue.length, value.length);
        }

        date = currentDate;
        System.arraycopy(currentValue, 0, value, 0, currentValue.length);

        // notifies monitors
        for (final Monitor monitor : monitors) {
            monitor.valueChanged(this);
        }

    }

}
