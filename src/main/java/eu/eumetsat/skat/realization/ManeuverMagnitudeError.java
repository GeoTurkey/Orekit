/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.random.RandomGenerator;
import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * Class for introducing random realization errors in magnitude to maneuvers.
 * <p>
 * Maneuver realization errors in magnitude are multiplying factors that apply
 * to the norm of the maneuver velocity increment. The multiplying factor is
 * drawn from a random number generator using a normal (Gaussian) law with zero
 * mean and user provided standard deviation.
 * </p>
 * <p>
 * As the multiplying factor may be different for in-plane and out-of-plane
 * maneuvers, a direction filter is applied: it the maneuver direction in
 * spacecraft frame is aligned with the filter direction or its reverse
 * (within a 10 degrees tolerance), then the error is applied to the maneuver,
 * otherwise the maneuver is left untouched. The user can therefore set up
 * several instances, each one using the thrust direction of a different set
 * of thrusters as its filtering direction and each one using its specific
 * error factor.
 * </p>
 * <p>
 * So for example given the maneuver (-0.02m/s, 0.0m/s, 0.001m/s) and
 * a filtering direction equal to the X axis, the maneuver is considered
 * affected by the error (it is at 177.14 degrees from the X axis, i.e.
 * less than 10 degrees from its opposite). With a standard deviation
 * of 0.05 (i.e. 5%), we may get a random value of -0.009 for example
 * and in this case the realized maneuver woud be (-0.01982m/s, 0.0m/s,
 * 0.000991m/s) which corresponds to the original vector multiplied by
 * 1 - 0.009. The same maneuver would not be affected by an error configured
 * with a filtering direction along the Y or Z axes.
 * </p>
 * @author Luc Maisonobe
 */
public class ManeuverMagnitudeError implements ScenarioComponent {

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Indicator for applying this error to in-plane maneuvers. */
    private final boolean inPlane;

    /** Indicator for applying this error to out-of-plane maneuvers. */
    private final boolean outOfPlane;

    /** Standard deviation of the multiplying error factor. */
    private final double standardDeviation;

    /** Random generator to use for evaluating the error factor. */
    private final RandomGenerator generator;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param inPlane if true, the error applies to in-plane maneuvers
     * @param outOfPlane if true, the error applies to out-of-plane maneuvers
     * @param standardDeviation standard deviation of the multiplying error factor
     * (for example 0.05 for a 5% error)
     * @param generator random generator to use for evaluating the error factor
     */
    public ManeuverMagnitudeError(final int[] spacecraftIndices,
                                  final boolean inPlane, final boolean outOfPlane,
                                  final double standardDeviation,
                                  final RandomGenerator generator) {
        this.spacecraftIndices  = spacecraftIndices.clone();
        this.inPlane            = inPlane;
        this.outOfPlane         = outOfPlane;
        this.standardDeviation  = standardDeviation;
        this.generator          = generator;
    }

    /** {@inheritDoc} */
    public void setCycleEnd(final AbsoluteDate cycleEnd) {
        // nothing to do here
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException, SkatException {

        ScenarioState[] updated = originals.clone();

        for (int i = 0; i < spacecraftIndices.length; ++i) {

            // select the current spacecraft affected by this component
            final int index = spacecraftIndices[i];
            final List<ScheduledManeuver> rawManeuvers = originals[index].getManeuvers();
            if (rawManeuvers == null) {
                throw new SkatException(SkatMessages.NO_MANEUVERS_STATE,
                                        originals[index].getName(), originals[index].getCyclesNumber());
            }

            // prepare a list for holding the modified maneuvers
            List<ScheduledManeuver> modified = new ArrayList<ScheduledManeuver>();

            // modify the maneuvers
            for (final ScheduledManeuver maneuver : rawManeuvers) {
                if ((inPlane && maneuver.isInPlane()) || (outOfPlane && !(maneuver.isInPlane()))) {
                    // the maneuver is affected by the error
                    final double errorFactor = 1.0 + standardDeviation * generator.nextGaussian();
                    modified.add(new ScheduledManeuver(maneuver.isInPlane(),
                                                       maneuver.getDate(),
                                                       new Vector3D(errorFactor, maneuver.getDeltaV()),
                                                       maneuver.getIsp()));
                } else {
                    // the maneuver is immune to the error
                    modified.add(maneuver);
                }
            }

            // update the state
            updated[index] = originals[index].updateManeuvers(modified);

        }

        // return an updated states
        return updated;

    }

}
