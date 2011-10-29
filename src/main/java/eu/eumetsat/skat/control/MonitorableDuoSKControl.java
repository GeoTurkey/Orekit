/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.HashSet;
import java.util.Set;

import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.utils.MonitorDuo;
import eu.eumetsat.skat.utils.MonitorableDuo;


/**
 * Implementation of the {@link MonitorDuo} interface for monitoring {@link SKControl control laws}.
 * <p>
 * Monitoring is done by interposing an instance that intercepts calls to the wrapped
 * {@link SKControl control law} and can therefore pickup the changed values when
 * {@link SKControl#getAchievedValue()} is called.
 * </p>
 * @author Luc Maisonobe
 */
public class MonitorableDuoSKControl implements MonitorableDuo, SKControl {

    /** Monitored control law. */
    private final SKControl control;

    /** Current date. */
    private AbsoluteDate date;

    /** Monitors interested in monitoring this value. */
    private final Set<MonitorDuo> monitors;

    /** Current value. */
    private double[] value;

    /** Simple constructor.
     * @param name name of the control law
     * @param scale of the control law
     * @param targetValue target value of the control law
     */
    public MonitorableDuoSKControl(final SKControl control) {
        this.control  = control;
        this.date     = AbsoluteDate.PAST_INFINITY;
        this.monitors = new HashSet<MonitorDuo>();
        this.value    = new double[1];
    }

    /** {@inheritDoc} */
    public void register(final int nbSpacecrafts, final MonitorDuo monitor) {
        monitors.add(monitor);
        monitor.startMonitoring(this);
    }

    /** {@inheritDoc} */
    public AbsoluteDate getDate() {
        return date;
    }

    /** {@inheritDoc} */
    public double[] getValue(int spacecraftIdx1, int spacecraftIdx2) {
        return value;
    }

    /** {@inheritDoc} */
    public String getName() {
        return control.getName();
    }

    /** {@inheritDoc} */
    public String getControlledSpacecraftName() {
        return control.getControlledSpacecraftName();
    }

    /** {@inheritDoc} */
    public String getReferenceSpacecraftName() {
        return control.getReferenceSpacecraftName();
    }

    /** {@inheritDoc} */
    public double getScale() {
        return control.getScale();
    }

    /** {@inheritDoc} */
    public double getTargetValue() {
        return control.getTargetValue();
    }

    /** {@inheritDoc} */
    public double getAchievedValue() {
        // monitor the residuals
        final double achieved = control.getAchievedValue();
        value[0] = achieved - getTargetValue();
        return achieved;
    }

    /** {@inheritDoc} */
    public boolean isConstrained() {
        return control.isConstrained();
    }

    /** {@inheritDoc} */
    public double getMin() {
        return control.getMin();
    }

    /** {@inheritDoc} */
    public double getMax() {
        return control.getMax();
    }

    /** {@inheritDoc} */
    public EventDetector getEventDetector() {
        return control.getEventDetector();
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return control.getStepHandler();
    }

}
