/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.random.RandomGenerator;
import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;

/**
 * Class for introducing random realization errors in date to maneuvers.
 * <p>
 * Maneuver realization errors in date are offsets that apply directly to the
 * maneuver date. The offset is drawn from a random number generator using a
 * normal (Gaussian) law with zero mean and user provided standard deviation.
 * </p>
 * @author Luc Maisonobe
 */
public class ManeuverDateError implements ScenarioComponent {

    /** Index of the spacecraft managed by this component. */
    private final int spacecraftIndex;

    /** Indicator for applying this error to in-plane maneuvers. */
    private final boolean inPlane;

    /** Indicator for applying this error to out-of-plane maneuvers. */
    private final boolean outOfPlane;

    /** Standard deviation of the multiplying error factor. */
    private final double standardDeviation;

    /** Random generator to use for evaluating the error factor. */
    private final RandomGenerator generator;

    /** Simple constructor.
     * @param spacecraftIndex index of the spacecraft managed by this component
     * @param inPlane if true, the error applies to in-plane maneuvers
     * @param outOfPlane if true, the error applies to out-of-plane maneuvers
     * @param standardDeviation standard deviation of the multiplying error factor
     * (for example 0.05 for a 5% error)
     * @param generator random generator to use for evaluating the error factor
     */
    public ManeuverDateError(final int spacecraftIndex,
                                  final boolean inPlane, final boolean outOfPlane,
                                  final double standardDeviation,
                                  final RandomGenerator generator) {
        this.spacecraftIndex    = spacecraftIndex;
        this.inPlane            = inPlane;
        this.outOfPlane         = outOfPlane;
        this.standardDeviation  = standardDeviation;
        this.generator          = generator;
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals, AbsoluteDate target)
        throws OrekitException {

        // prepare a list for holding the modified maneuvers
        List<ScheduledManeuver> modified =
                new ArrayList<ScheduledManeuver>(originals[spacecraftIndex].getPerformedManeuvers().size());

        // modify the maneuvers
        for (final ScheduledManeuver maneuver : originals[spacecraftIndex].getPerformedManeuvers()) {
            if ((inPlane && maneuver.isInPlane()) || (outOfPlane && !(maneuver.isInPlane()))) {
                // the maneuver is affected by the error
                final double offset = standardDeviation * generator.nextGaussian();
                modified.add(new ScheduledManeuver(maneuver.isInPlane(),
                                                   maneuver.getDate().shiftedBy(offset),
                                                   maneuver.getDeltaV(),
                                                   maneuver.getIsp()));
            } else {
                // the maneuver is immune to the error
                modified.add(maneuver);
            }
        }

        // return an updated state
        ScenarioState[] updated = originals.clone();
        updated[spacecraftIndex] = originals[spacecraftIndex].updatePerformedManeuvers(modified);
        return updated;

    }

}
