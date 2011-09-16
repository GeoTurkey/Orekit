/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

/**
 * Container for a scale and station-keeping control pair.
 * @author Luc Maisonobe
 */
class ScaledControl {

    /** Weight. */
    private final double scale;

    /** Control. */
    private final SKControl control;

    /** Simple constructor.
     * @param scale scale of the control
     * @param control station-keeping control
     */
    public ScaledControl(final double scale, final SKControl control) {
        this.scale   = scale;
        this.control = control;
    }

    /** Get the scale.
     * @return scale of the station-keeping control
     */
    public double getScale() {
        return scale;
    }

    /** Get the station-keeping control.
     * @return station-keeping control
     */
    public SKControl getControl() {
        return control;
    }

    /** Get the scaled residual squared.
     * <p>
     * The scaled residual squared is equal to:
     * ((control.{@link StationKeepingControl#getAchievedValue() getAchievedValue()}
     *  - control.{@link StationKeepingControl#getTargetValue() getTarget()})
     *  / scale)<sup>2</sup>
     * </p>
     */
    public double getScaledResidualSquared() {
        final double residual = control.getAchievedValue() - control.getTargetValue();
        final double scaledResidual = residual / scale;
        return scaledResidual * scaledResidual;
    }
}


