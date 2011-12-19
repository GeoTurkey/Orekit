/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import org.apache.commons.math.util.FastMath;



/**
 * Base implementation for station-keeping control laws.
 */
public abstract class AbstractSKControl implements SKControl {

    /** Name of the control law. */
    private final String name;

    /** Name of the controlled spacecraft. */
    private final String controlledName;

    /** Index of the controlled spacecraft. */
    private final int controlledIndex;

    /** Name of the reference spacecraft (may be null). */
    private final String referenceName;

    /** Index of the reference spacecraft (may be null). */
    private final int referenceIndex;

    /** Control target used to compute residuals. */
    private final double target;

    /** Minimal value for the residual. */
    private final double min;

    /** Maximal value for the residual. */
    private final double max;

    /** Indicator of constraint violation during the cycle. */
    private double limitsExceeded;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param referenceName name of the reference spacecraft
     * @param referenceIndex index of the reference spacecraft
     * @param target control target used to compute residuals
     * @param min minimal value for the residual
     * @param max maximal value for the residual
     */
    protected AbstractSKControl(final String name, final String controlledName,
                                final int controlledIndex, final String referenceName,
                                final int referenceIndex, final double target,
                                final double min, final double max) {
        this.name            = name;
        this.controlledName  = controlledName;
        this.controlledIndex = controlledIndex;
        this.referenceName   = referenceName;
        this.referenceIndex  = referenceIndex;
        this.target          = target;
        this.min             = min;
        this.max             = max;
    }

    /** {@inheritDoc} */
    public String getName() {
        return name;
    }

    /** {@inheritDoc} */
    public String getControlledSpacecraftName() {
        return controlledName;
    }

    /** {@inheritDoc} */
    public int getControlledSpacecraftIndex() {
        return controlledIndex;
    }

    /** {@inheritDoc} */
    public String getReferenceSpacecraftName() {
        return referenceName;
    }

    /** {@inheritDoc} */
    public int getReferenceSpacecraftIndex() {
        return referenceIndex;
    }

    /** Reset the limits checks.
     */
    protected void resetLimitsChecks() {
        limitsExceeded = 0;
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

    /** {@inheritDoc} */
    public double limitsExceeded() {
        return limitsExceeded;
    }

    /** Check if control limits are exceeded.
     * @param value current value of the control law
     */
    protected void checkLimits(final double value) {
        if (value < min) {
            limitsExceeded = FastMath.max(limitsExceeded, min - value);
        } else if (value > max) {
            limitsExceeded = FastMath.max(limitsExceeded, value - max);
        }
    }

}
