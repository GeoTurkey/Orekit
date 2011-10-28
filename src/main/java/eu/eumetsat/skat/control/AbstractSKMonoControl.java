/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.HashSet;
import java.util.Set;

import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.utils.MonitorMono;


/**
 * Base implementation of {@link SKMonoControl}.
 * @author Luc Maisonobe
 */
public abstract class AbstractSKMonoControl implements SKMonoControl {

    /** Name of the control law. */
    private final String name;

    /** Scale of the control law. */
    private final double scale;

    /** Target value of the control law. */
    private final double targetValue;

    /** Current date. */
    private AbsoluteDate date;

    /** Monitors interested in monitoring this value. */
    private final Set<MonitorMono> monitors;

    /** Current value. */
    private double[] value;

    /** Simple constructor.
     * @param name name of the control law
     * @param scale of the control law
     * @param targetValue target value of the control law
     */
    protected AbstractSKMonoControl(final String name, final double scale, final double targetValue) {
        this.name        = name;
        this.scale       = scale;
        this.targetValue = targetValue;
        this.date        = AbsoluteDate.PAST_INFINITY;
        this.monitors    = new HashSet<MonitorMono>();
        this.value       = new double[1];
    }

    /** {@inheritDoc} */
    public void register(final int nbSpacecrafts, final MonitorMono monitor) {
        monitors.add(monitor);
        monitor.startMonitoring(this);
    }

    /** {@inheritDoc} */
    public String getName() {
        return name;
    }

    /** {@inheritDoc} */
    public double getScale() {
        return scale;
    }

    /** {@inheritDoc} */
    public AbsoluteDate getDate() {
        return date;
    }

    /** {@inheritDoc} */
    public double getTargetValue() {
        return targetValue;
    }

    /** {@inheritDoc} */
    public double[] getValue(int spacecraftIdx) {
        return value;
    }

}
