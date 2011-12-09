/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.analysis.UnivariateFunction;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.optimization.GoalType;
import org.apache.commons.math.optimization.univariate.BrentOptimizer;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.analytical.ManeuverAdapterPropagator;
import org.orekit.propagation.sampling.OrekitStepHandlerMultiplexer;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.SKControl;
import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.OrekitWrapperException;
import eu.eumetsat.skat.utils.SkatException;

/**
 * Class for selecting one burn if it fulfills the constraints, two burns otherwise.
 * <p>
 * One burn should have already been optimized. The component checks if it fulfills
 * the control law constraints, and if so it let it alone. Otherwise, it split the
 * burn in two parts separated by one half orbit such that the overall velocity
 * increment remains the same, and optimizes the ratio between the two parts according
 * to the initial control laws.
 * </p>
 * @author Luc Maisonobe
 */
public class OneOrTwoBurnsSelector implements ScenarioComponent {

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Name of the single burn. */
    private final String burnName;

    /** End of the cycle. */
    private AbsoluteDate cycleEnd;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param burnName name of the single burn
     */
    public OneOrTwoBurnsSelector(final int[] spacecraftIndices, final String burnName)
        throws IllegalArgumentException {
        this.spacecraftIndices = spacecraftIndices.clone();
        this.burnName          = burnName;
    }

    /** {@inheritDoc} */
    public void setCycleEnd(final AbsoluteDate cycleEnd) {
        this.cycleEnd = cycleEnd;
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException, SkatException {

        ScenarioState[] updated = originals.clone();

        for (int i = 0; i < spacecraftIndices.length; ++i) {

            // select the current spacecraft affected by this component
            final int index = spacecraftIndices[i];

            // select the single burn that may be split
            final ScheduledManeuver singleBurn = selectBurn(originals[index].getManeuvers());
            if (singleBurn == null) {
                break;
            }

            // check if the single burn fulfills constraints as is
            final boolean fulfilled = constraintsFulfilled(new ManeuverAdapterPropagator(singleBurn.getTrajectory()),
                                                           singleBurn.getControlLaws(),
                                                           originals[index]);
            if (!fulfilled) {

                // prepare a list for holding the modified maneuvers
                List<ScheduledManeuver> modified = new ArrayList<ScheduledManeuver>();

                // set up the preserved maneuvers
                for (final ScheduledManeuver maneuver : originals[index].getManeuvers()) {
                    if (!maneuver.getName().equals(burnName)) {
                        modified.add(maneuver);
                    }
                }

                // remove the modified single burn from the reference propagator
                final ManeuverAdapterPropagator reference =
                        new ManeuverAdapterPropagator(singleBurn.getTrajectory());
                reference.addManeuver(singleBurn.getDate(), singleBurn.getDeltaV().negate(), singleBurn.getIsp());

                // optimize the dV ratio between the two parts of the split maneuver
                final AbsoluteDate tA      = singleBurn.getDate();
                final AbsoluteDate tB      = tA.shiftedBy(0.5 * reference.propagate(tA).getKeplerianPeriod());
                final UnivariateFunction f = new UnivariateFunction() {
                    
                    /** {@inheritDoc} */
                    public double value(double x) {
                        try {

                            final ManeuverAdapterPropagator map = new ManeuverAdapterPropagator(reference);
                            map.addManeuver(tA, new Vector3D(x,     singleBurn.getDeltaV()), singleBurn.getIsp());
                            map.addManeuver(tB, new Vector3D(1 - x, singleBurn.getDeltaV()), singleBurn.getIsp());
                            monitorControlLaws(map, singleBurn.getControlLaws(), originals[index]);

                            double sum = 0;
                            for (final SKControl s : singleBurn.getControlLaws()) {
                                final double residual = s.getAchievedValue() - s.getTargetValue();
                                final double scaledResidual = residual / s.getScalingDivisor();
                                sum += scaledResidual * scaledResidual;
                            }

                            return sum;

                        } catch (OrekitException oe) {
                            throw new OrekitWrapperException(oe);
                        }
                    }

                };
                final BrentOptimizer optimizer = new BrentOptimizer(1.0e-3, 1.0e-3);
                final double optimumRatio = optimizer.optimize(1000, f, GoalType.MINIMIZE, 0.0, 1.0).getPoint();

                // add the two parts of the split maneuver
                final ScheduledManeuver partA =
                        new ScheduledManeuver(singleBurn.getName() + "(A)", singleBurn.isInPlane(),
                                              tA, new Vector3D(optimumRatio, singleBurn.getDeltaV()),
                                              singleBurn.getThrust(), singleBurn.getIsp(),
                                              reference, singleBurn.getControlLaws(), false);
                modified.add(partA);
                reference.addManeuver(partA.getDate(), partA.getDeltaV(), partA.getIsp());
                final ScheduledManeuver partB =
                        new ScheduledManeuver(singleBurn.getName() + "(B)", singleBurn.isInPlane(),
                                              tB, new Vector3D(1 - optimumRatio, singleBurn.getDeltaV()),
                                              singleBurn.getThrust(), singleBurn.getIsp(),
                                              reference, singleBurn.getControlLaws(), false);
                modified.add(partB);
                reference.addManeuver(partB.getDate(), partB.getDeltaV(), partB.getIsp());

                // update the state
                updated[index] = originals[index].updateManeuvers(modified);

            }

        }

        // return an updated states
        return updated;

    }

    /** Select the burn to split.
     * @param maneuvers maneuvers list
     * @return selected burn (null if not found)
     */
    private ScheduledManeuver selectBurn(final List<ScheduledManeuver> maneuvers)
        throws SkatException {

        if (maneuvers != null) {
            for (final ScheduledManeuver maneuver : maneuvers) {
                if (maneuver.getName().equals(burnName)) {
                    return maneuver;
                }
            }
        }

        return null;

    }

    /** Check if a maneuvers configuration fulfills the control laws constraints.
     * @param propagator to use (already includes the maneuvers)
     * @param controls control laws to monitor
     * @param scenario state
     * @return true if the constraints of the laws linked to the maneuver are fulfilled
     * @exception OrekitException if propagation fails
     */
    private boolean constraintsFulfilled(final Propagator propagator,
                                         final List<SKControl> controls,
                                         final ScenarioState state)
        throws OrekitException {

        monitorControlLaws(propagator, controls, state);

        // check constraints
        for (final SKControl controlLaw : controls) {
            if (controlLaw.limitsExceeded()) {
                return false;
            }
        }
        return true;

    }

    /** Monitor control laws.
     * @param propagator to use (already includes the maneuvers)
     * @param controls control laws to monitor
     * @param scenario state
     * @exception OrekitException if propagation fails
     */
    private void monitorControlLaws(final Propagator propagator,
                                    final List<SKControl> controls,
                                    final ScenarioState state)
        throws OrekitException {

        final AbsoluteDate cycleStart = state.getEstimatedState().getDate();
        for (final SKControl controlLaw : controls) {
            final List<ScheduledManeuver> maneuvers = state.getManeuvers();
            controlLaw.initializeRun(maneuvers.toArray(new ScheduledManeuver[maneuvers.size()]),
                                     propagator, cycleStart, cycleEnd, 1);
        }

        // register the control law handlers to the propagator
        final OrekitStepHandlerMultiplexer multiplexer = new OrekitStepHandlerMultiplexer();
        for (final SKControl controlLaw : controls) {
            if (controlLaw.getEventDetector() != null) {
                propagator.addEventDetector(controlLaw.getEventDetector());
            }
            if (controlLaw.getStepHandler() != null) {
                multiplexer.add(controlLaw.getStepHandler());
            }
        }
        propagator.setMasterMode(multiplexer);


        // perform propagation
        propagator.propagate(cycleStart, cycleEnd);

    }

}
