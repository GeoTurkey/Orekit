/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.orekit.propagation.analytical.ManeuverAdapterPropagator;

import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;



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

    /** Minimal value for the residual. */
    private final double min;

    /** Maximal value for the residual. */
    private final double max;

    /** Indicator of constraint violation during the cycle. */
    private double margins;

    /** Simple constructor.
     * @param name name of the control law
     * @param controlledName name of the controlled spacecraft
     * @param controlledIndex index of the controlled spacecraft
     * @param referenceName name of the reference spacecraft
     * @param referenceIndex index of the reference spacecraft
     * @param min minimal value for the residual
     * @param max maximal value for the residual
     */
    protected AbstractSKControl(final String name,
                                final String controlledName, final int controlledIndex,
                                final String referenceName, final int referenceIndex,
                                final double min, final double max) {
        this.name            = name;
        this.controlledName  = controlledName;
        this.controlledIndex = controlledIndex;
        this.referenceName   = referenceName;
        this.referenceIndex  = referenceIndex;
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

    /** Reset the margins checks.
     */
    protected void resetMarginsChecks() {
        margins = Double.POSITIVE_INFINITY;
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
    public double getMargins() {
        return margins;
    }

    /** Check if control limits are exceeded.
     * @param value current value of the control law
     */
    protected void checkMargins(final double value) {
        if (isConstrained()) {
            margins = FastMath.min(margins, FastMath.min(value - min, max - value));
        } else {
            margins = 0.0;
        }
    }

    /** Change the trajectory of some maneuvers.
     * @param maneuvers maneuvers array
     * @param from index of the first maneuver to update (included)
     * @param to index of the last maneuver to update (excluded)
     * @param trajectory trajectory to use for maneuvers
     */
    protected void changeTrajectory(final ScheduledManeuver[] maneuvers,
                                    final int from, final int to,
                                    final ManeuverAdapterPropagator adapterPropagator) {
        for (int i = from; i < to; ++i) {
            maneuvers[i] = new ScheduledManeuver(maneuvers[i].getModel(), maneuvers[i].getDate(),
                                                 maneuvers[i].getDeltaV(), maneuvers[i].getThrust(),
                                                 maneuvers[i].getIsp(), adapterPropagator,
                                                 maneuvers[i].isReplanned());
        }
    }

    /** Distribute a velocity increment change over non-saturated maneuvers.
     * @param dV velocity increment change to distribute
     * @param dT date change to apply to all maneuvers
     * @param model model of the maneuvers to change
     * @param maneuvers array of maneuvers to change
     * @param adapterPropagator propagator to use for maneuvers
     */
    protected void distributeDV(final double dV, final double dT,
                                final TunableManeuver model, final ScheduledManeuver[] maneuvers,
                                final ManeuverAdapterPropagator adapterPropagator) {

        final double inf = model.getDVInf();
        final double sup = model.getDVSup();

        double remaining = dV;
        while (FastMath.abs(remaining) > model.getEliminationThreshold()) {

            // identify the maneuvers that can be changed
            final List<Integer> nonSaturated = new ArrayList<Integer>(maneuvers.length);
            for (int i = 0; i < maneuvers.length; ++i) {
                if (maneuvers[i].getName().equals(model.getName()) &&
                    ((dV < 0 && maneuvers[i].getSignedDeltaV() > inf) ||
                     (dV > 0 && maneuvers[i].getSignedDeltaV() < sup))) {
                    nonSaturated.add(i);
                }
            }

            if (nonSaturated.isEmpty()) {
                // we cannot do anything more
                return;
            }

            // distribute the remaining dV evenly
            final double dVPart = remaining / nonSaturated.size();
            for (final int i : nonSaturated) {
                final double original = maneuvers[i].getSignedDeltaV();
                final double changed  = FastMath.max(inf, FastMath.min(sup, original + dVPart));
                remaining   -= changed - original;
                maneuvers[i] = new ScheduledManeuver(maneuvers[i].getModel(),
                                                     maneuvers[i].getDate().shiftedBy(dT),
                                                     new Vector3D(changed, model.getDirection()),
                                                     maneuvers[i].getThrust(),
                                                     maneuvers[i].getIsp(),
                                                     adapterPropagator,
                                                     maneuvers[i].isReplanned());
            }

        }

    }

}
