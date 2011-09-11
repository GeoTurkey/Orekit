/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

/**
 * Container for a scale and station-keeping goal pair.
 * @author Luc Maisonobe
 */
class ScaledGoal {

    /** Weight. */
    private final double scale;

    /** Goal. */
    private final StationKeepingGoal goal;

    /** Simple constructor.
     * @param scale scale of the goal
     * @param goal station-keeping goal
     */
    public ScaledGoal(final double scale, final StationKeepingGoal goal) {
        this.scale = scale;
        this.goal  = goal;
    }

    /** Get the scale.
     * @return scale of the station-keeping goal
     */
    public double getScale() {
        return scale;
    }

    /** Get the station-keeping goal.
     * @return station-keeping goal
     */
    public StationKeepingGoal getGoal() {
        return goal;
    }

    /** Get the scaled residual squared.
     * <p>
     * The scaled residual squared is equal to:
     * ((goal.{@link StationKeepingGoal#getAchievedValue() getAchievedValue()}
     *  - goal.{@link StationKeepingGoal#getTarget() getTarget()})
     *  / scale)<sup>2</sup>
     * </p>
     */
    public double getScaledResidualSquared() {
        final double residual = goal.getAchievedValue() - goal.getTarget();
        final double scaledResidual = residual / scale;
        return scaledResidual * scaledResidual;
    }
}


