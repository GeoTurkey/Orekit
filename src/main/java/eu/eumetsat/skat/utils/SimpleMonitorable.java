/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.util.HashSet;
import java.util.Set;

import org.orekit.time.AbsoluteDate;


/**
 * Simple implementation of the {@link MonitorMono} and {@link MonitorDuo} interfaces.
 * @author Luc Maisonobe
 */
public class SimpleMonitorable implements MonitorableMono, MonitorableDuo {

    /** Name of the monitored value. */
    private String name;

    /** Current date. */
    private AbsoluteDate date;

    /** Monitors interested in monitoring this value. */
    private final Set<MonitorMono> monitorsMono;

    /** Monitors interested in monitoring this value. */
    private final Set<MonitorDuo> monitorsDuo;

    /** Current value. */
    private double[] value;

    /** Simple constructor.
     * @param dimension dimension of the value
     * @param name of the monitored value
     */
    public SimpleMonitorable(final int dimension, final String name) {
        this.name         = name;
        this.date         = AbsoluteDate.PAST_INFINITY;
        this.monitorsMono = new HashSet<MonitorMono>();
        this.monitorsDuo  = new HashSet<MonitorDuo>();
        this.value        = new double[dimension];
    }

    /** {@inheritDoc} */
    public void register(final int nbSpacecrafts, final MonitorMono monitor) {
        monitorsMono.add(monitor);
        monitor.startMonitoring(this);
    }

    /** {@inheritDoc} */
    public void register(final int nbSpacecrafts, final MonitorDuo monitor) {
        monitorsDuo.add(monitor);
        monitor.startMonitoring(this);
    }

    /** {@inheritDoc} */
    public AbsoluteDate getDate() {
        return date;
    }

    /** {@inheritDoc} */
    public double[] getValue(int spacecraftIdx) {
        return value;
    }

    /** {@inheritDoc} */
    public double[] getValue(int spacecraftIdx1, int spacecraftIdx2) {
        return value;
    }

    /** {@inheritDoc} */
    public String getName() {
        return name;
    }

    /** Set a sample value.
     * @param date date of the monitored value sample
     * @param value monitored value sample
     */
    public void setSampledValue(final AbsoluteDate sampleDate, final double[] sampleValue) {

        this.date = sampleDate;
        System.arraycopy(sampleValue, 0, value, 0, value.length);

        // notifies monitors
        for (final MonitorMono monitor : monitorsMono) {
            monitor.valueChanged(this);
        }
        for (final MonitorDuo monitor : monitorsDuo) {
            monitor.valueChanged(this);
        }

    }

}
