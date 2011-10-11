/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.random.RandomGenerator;
import org.apache.commons.math.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;

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
 * of 0.005 (i.e. 5%), we may get a random value of -0.009 for example
 * and in this case the realized maneuver woud be (-0.01982m/s, 0.0m/s,
 * 0.000991m/s) which corresponds to the original vector multiplied by
 * 1 - 0.009. The same maneuver would not be affected by an error configured
 * with a filtering direction along the Y or Z axes.
 * </p>
 * @author Luc Maisonobe
 */
public class ManeuverMagnitudeError implements ScenarioComponent {

    /** Angular threshold for maneuver direction filtering. */
    private static final double ALIGNMENT_THRESHOLD = FastMath.toRadians(10.0);

    /** Index of the spacecraft managed by this component. */
    private final int spacecraftIndex;

    /** Maneuvers direction to which this error applies. */
    private final Vector3D filteringDirection;

    /** Standard deviation of the multiplying error factor. */
    private final double standardDeviation;

    /** Random generator to use for evaluating the error factor. */
    private final RandomGenerator generator;

    /** Simple constructor.
     * @param spacecraftIndex index of the spacecraft managed by this component
     * @param filteringDirection filter used to decide if the instance should be
     * applied to a maneuver or not
     * @param standardDeviation standard deviation of the multiplying error factor
     * (for example 0.05 for a 5% error)
     * @param generator random generator to use for evaluating the error factor
     */
    public ManeuverMagnitudeError(final int spacecraftIndex,
                                  final Vector3D filteringDirection,
                                  final double standardDeviation,
                                  final RandomGenerator generator) {
        this.spacecraftIndex    = spacecraftIndex;
        this.filteringDirection = filteringDirection;
        this.standardDeviation  = standardDeviation;
        this.generator          = generator;
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateState(final ScenarioState[] originals, AbsoluteDate target)
        throws OrekitException {

        // prepare a list for holding the modified maneuvers
        List<ImpulseManeuver> modified =
                new ArrayList<ImpulseManeuver>(originals[spacecraftIndex].getPerformedManeuvers().size());

        // modify the maneuvers
        for (final ImpulseManeuver maneuver : originals[spacecraftIndex].getPerformedManeuvers()) {
            final double angle = Vector3D.angle(filteringDirection, maneuver.getDeltaVSat());
            if ((angle < ALIGNMENT_THRESHOLD) || (angle > FastMath.PI - ALIGNMENT_THRESHOLD)) {
                // the maneuver is affected by the error
                final double errorFactor = 1.0 + standardDeviation * generator.nextGaussian();
                modified.add(new ImpulseManeuver(maneuver.getTrigger(),
                                                 new Vector3D(errorFactor, maneuver.getDeltaVSat()),
                                                 maneuver.getIsp()));
            } else {
                // the maneuver is immune to the error
                modified.add(maneuver);
            }
        }

        // return an updated state
        ScenarioState[] updated = originals.clone();
        updated[spacecraftIndex] = new ScenarioState(originals[spacecraftIndex].getRealState(),
                                                     originals[spacecraftIndex].getEstimatedState(),
                                                     originals[spacecraftIndex].getTheoreticalManeuvers(),
                                                     modified);
        return updated;

    }

}
