/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;


/**
 * Base implementation for station-keeping control laws.
 */
public abstract class AbstractSKControl implements SKControl {

    /** Name of the control law. */
    private final String name;

    /** Scale of the control law. */
    private final double scale;

    /** Name of the controlled spacecraft. */
    private final String controlled;

    /** Name of the reference spacecraft (may be null). */
    private final String reference;

    /** Control target used to compute residuals. */
    private final double target;

    /** Minimal value for the residual. */
    private final double min;

    /** Maximal value for the residual. */
    private final double max;

    /** Simple constructor.
     * @param name name of the control law
     * @param scale of the control law
     * @param controlled name of the controlled spacecraft
     * @param reference name of the reference spacecraft
     * @param target control target used to compute residuals
     * @param min minimal value for the residual
     * @param max maximal value for the residual
     */
    protected AbstractSKControl(final String name, final double scale,
                                final String controlled, final String reference,
                                final double target,
                                final double min, final double max) {
        this.name       = name;
        this.scale      = scale;
        this.controlled = controlled;
        this.reference  = reference;
        this.target     = target;
        this.min        = min;
        this.max        = max;
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
    public String getControlledSpacecraftName() {
        return controlled;
    }

    /** {@inheritDoc} */
    public String getReferenceSpacecraftName() {
        return reference;
    }

    /** {@inheritDoc} */
    public double getTargetValue() {
        return target;
    }

    /** {@inheritDoc} */
    public boolean isConstrained() {
        return (min != Double.NEGATIVE_INFINITY) || (max != Double.POSITIVE_INFINITY);
    }

    /** {@inheritDoc} */
    public double getMin() {
        return min;
    }

    /** {@inheritDoc} */
    public double getMax() {
        return max;
    }

}
