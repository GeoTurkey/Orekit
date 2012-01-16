/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.scenario;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TimeZone;

import org.apache.commons.math.util.FastMath;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBody;
import org.orekit.errors.OrekitException;
import org.orekit.frames.TopocentricFrame;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.sampling.OrekitStepHandlerMultiplexer;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;

import eu.eumetsat.skat.control.SKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.MonitorableDuoSKData;
import eu.eumetsat.skat.utils.MonitorableMonoSKData;
import eu.eumetsat.skat.utils.SimpleMonitorable;
import eu.eumetsat.skat.utils.SkatException;

/** Station-Keeping scenario.
 * <p>
 * A station-keeping scenario is a simple list of cycle components
 * that are run one after the other at each cycle, and then the cycle
 * is repeated.
 * </p>
 */
public class Scenario implements ScenarioComponent {

    /** Scenario components. */
    private final List<ScenarioComponent> components;

    /** Output step for monitoring. */
    private final double outputstep;

    /** Earth model. */
    private final BodyShape earth;

    /** Cycle end date. */
    private AbsoluteDate cycleEnd;

    /** Reference ground location. */
    private TopocentricFrame groundLocation;

    /** Mono-spacecraft monitorables. */
    private List<MonitorableMonoSKData> monitorablesMono;

    /** Duo-spacecrafts monitorables. */
    private List<MonitorableDuoSKData> monitorablesDuo;

    /** Maneuvers monitorables. */
    private final Map<String, SimpleMonitorable> maneuversMonitorables;

    /** Map for control laws residuals monitoring. */
    private Map<SKControl, SimpleMonitorable> controlsMargins;

    /** Map for control laws violations monitoring. */
    private Map<SKControl, SimpleMonitorable> controlsViolations;

    /** Maneuvers output. */
    private PrintStream maneuversOutput;

    /** Cycle duration. */
    private double cycleDuration;

    /** Simple constructor.
     * <p>
     * Create an empty scenario without any components. Components
     * must be added by calling {@link #addComponent(ScenarioComponent)}.
     * </p>
     * @param outputStep output step for monitoring (s)
     * @param earth Earth model
     * @param sun Sun model
     * @param groundLocation reference ground location
     * @param monitorablesMono list of monitorables for mono-spacecraft
     * @param monitorablesDuo list of monitorables for duo-spacecrafts
     * @param maneuversMonitorables maneuvers monitorables
     * @param controlsMargins map for control laws residuals monitoring
     * @param controlsViolations map for control laws violations monitoring
     * @param maneuversOutput maneuves output stream
     */
    public Scenario(final double outputStep,
                    final BodyShape earth, final CelestialBody sun,
                    final TopocentricFrame groundLocation,
                    final List<MonitorableMonoSKData> monitorablesMono,
                    final List<MonitorableDuoSKData> monitorablesDuo,
                    final Map<String, SimpleMonitorable> maneuversMonitorables,
                    final Map<SKControl, SimpleMonitorable> controlsMargins,
                    final Map<SKControl, SimpleMonitorable> controlsViolations,
                    final PrintStream maneuversOutput) {
        this.components            = new ArrayList<ScenarioComponent>();
        this.outputstep            = outputStep;
        this.earth                 = earth;
        this.groundLocation        = groundLocation;
        this.monitorablesMono      = monitorablesMono;
        this.monitorablesDuo       = monitorablesDuo;
        this.maneuversMonitorables = maneuversMonitorables;
        this.controlsMargins       = controlsMargins;
        this.controlsViolations    = controlsViolations;
        this.maneuversOutput       = maneuversOutput;
        this.cycleDuration         = 0.0;
    }

    /** Add a cycle component.
     * <p>
     * Cycle components must be added in simulation order.
     * </p>
     * @param component cycle component
     */
    public void addComponent(final ScenarioComponent component) {
        components.add(component);
    }

    /** {@inheritDoc} */
    public void setCycleDuration(final double cycleDuration) {
        this.cycleDuration = cycleDuration;
    }

    /** {@inheritDoc} */
    public void setCycleEnd(final AbsoluteDate cycleEnd) {
        this.cycleEnd = cycleEnd;
    }

    /** {@inheritDoc}
     * <p>The scenario will be run in loops until the target date
     * is reached. At each iteration of the loop, the target date of
     * the iteration will be set according to the cycle duration set at
     * construction.</p>
     */
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException, SkatException {

        ScenarioState[] states = originals.clone();
        AbsoluteDate iterationTarget;
        do {

            // set target date for iteration using cycle duration
            final AbsoluteDate startDate = states[0].getRealState().getDate();
            iterationTarget = startDate.shiftedBy(cycleDuration);
            if (iterationTarget.compareTo(cycleEnd) > 0) {
                iterationTarget = cycleEnd;
            }

            final TimeScale utc = TimeScalesFactory.getUTC();
            final Date now = Calendar.getInstance(TimeZone.getTimeZone("Etc/UTC")).getTime();
            System.out.println(new AbsoluteDate(now, utc).toString(utc) +
                               ": starting cycle " + states[0].getCyclesNumber() + " " +
                               startDate + " -> " + iterationTarget);

            // run all components of the scenario in order
            for (final ScenarioComponent component : components) {
                component.setCycleEnd(iterationTarget);
                states = component.updateStates(states);
            }

            performMonitoring(states);

            // prepare next cycle
            for (int i = 0; i < states.length; ++i) {
                states[i] = states[i].updateCyclesNumber(states[i].getCyclesNumber() + 1);
                for (final String name : states[i].getManeuversNames()) {
                    states[i] = states[i].updateManeuverStats(name,
                                                              states[i].getManeuversNumber(name),
                                                              0.0,
                                                              states[i].getManeuversTotalDV(name));

                    // monitor maneuver at cycle start
                    SimpleMonitorable monitorable = maneuversMonitorables.get(name);
                    monitorable.setSampledValue(states[0].getRealState().getDate(),
                                                new double[] {
                                                   states[i].getManeuversNumber(name),
                                                   states[i].getManeuversCycleDV(name),
                                                   states[i].getManeuversTotalDV(name)
                                                });
                }
                states[i] = states[i].updateEstimatedState(null);
                states[i] = states[i].updateManeuvers(new ArrayList<ScheduledManeuver>());
            }

        } while (cycleEnd.durationFrom(iterationTarget) > 1.0);

        return states;

    }

    /** Perform monitoring.
     * @param states scenario states
     * @exception OrekitException if propagation fails
     * @exception SkatException if there are no maneuvers
     */
    private void performMonitoring(ScenarioState[] states)
        throws OrekitException, SkatException {

        AbsoluteDate tMin = states[0].getPerformedEphemeris().getMinDate();
        AbsoluteDate tMax = states[0].getPerformedEphemeris().getMaxDate();

        // monitor control laws margins and violations
        Set<SKControl> controls = new HashSet<SKControl>();
        controls.addAll(controlsMargins.keySet());
        controls.addAll(controlsViolations.keySet());
        for (int i = 0; i < states.length; ++i) {

            final BoundedPropagator propagator = states[i].getPerformedEphemeris();
            propagator.clearEventsDetectors();

            for (final SKControl controlLaw : controls) {
                final List<ScheduledManeuver> maneuvers = states[i].getManeuvers();
                if (maneuvers == null) {
                    controlLaw.initializeRun(0, new ScheduledManeuver[0], propagator,
                                             new ArrayList<ScheduledManeuver>(), tMin, tMax);
                } else {
                    controlLaw.initializeRun(0, maneuvers.toArray(new ScheduledManeuver[maneuvers.size()]),
                                             propagator, new ArrayList<ScheduledManeuver>(), tMin, tMax);
                }
            }

            final OrekitStepHandlerMultiplexer multiplexer = new OrekitStepHandlerMultiplexer();

            // register the control law handlers to the propagator
            for (final SKControl controlLaw : controls) {
                if (i == controlLaw.getControlledSpacecraftIndex()) {
                    if (controlLaw.getEventDetector() != null) {
                        propagator.addEventDetector(controlLaw.getEventDetector());
                    }
                    if (controlLaw.getStepHandler() != null) {
                        multiplexer.add(controlLaw.getStepHandler());
                    }
                }
            }
            propagator.setMasterMode(multiplexer);

            final double safetyMargin = 1.0e-3;
            propagator.propagate(tMin.shiftedBy(safetyMargin), tMax.shiftedBy(-safetyMargin));

        }

        for (final Map.Entry<SKControl, SimpleMonitorable> entry : controlsMargins.entrySet()) {
            final double value = entry.getKey().getMargins();
            entry.getValue().setSampledValue(tMin, new double[] { value });
        }

        for (final Map.Entry<SKControl, SimpleMonitorable> entry : controlsViolations.entrySet()) {
            double[] v = entry.getValue().getValue(-1).clone();
            if (entry.getKey().getMargins() < 0) {
                // this cycle has lead to constraints violations, increment counter
                v[0] += 1;
            }
            entry.getValue().setSampledValue(tMin, v);
        }

        // monitor data
        final double safetyMargin = 1.0e-3;
        AbsoluteDate date = tMin.shiftedBy(safetyMargin);
        while (date.compareTo(tMax) <= 0) {

            // check for maneuvers that have occurred
            updatePendingManeuvers(date, states);

            // perform monitoring
            for (final MonitorableMonoSKData monitorable : monitorablesMono) {
                monitorable.update(date, states, earth);
            }
            for (final MonitorableDuoSKData monitorable : monitorablesDuo) {
                monitorable.update(date, states, earth, groundLocation);
            }

            // go to next step, adjusting the last step to be slightly before the limit
            if (tMax.durationFrom(date) > outputstep + safetyMargin) {
                date = date.shiftedBy(outputstep);
            } else if (FastMath.abs(tMax.durationFrom(date)) > 2 * safetyMargin) {
                date = tMax.shiftedBy(-safetyMargin);
            } else {
                date = AbsoluteDate.FUTURE_INFINITY;
            }

        }

    }

    /** Update the maneuvers state up to current date.
     * @param date current date
     * @param states states array to update
     * @exception OrekitException if a maneuver cannot be checked
     * @exception SkatException if there are no maneuvers
     */
    private void updatePendingManeuvers(final AbsoluteDate date, final ScenarioState[] states)
        throws OrekitException, SkatException {

        final AbsoluteDate previous = date.shiftedBy(-outputstep);

        for (int i = 0; i < states.length; ++i) {
            if (states[i].getManeuvers() != null) {
                for (final ScheduledManeuver maneuver : states[i].getManeuvers()) {

                    final AbsoluteDate maneuverDate = maneuver.getDate();

                    if ((maneuverDate.compareTo(previous) > 0) && (maneuverDate.compareTo(date) <= 0)) {

                        // the maneuver occurred during last step, take it into account
                        final double dv      = maneuver.getDeltaV().getNorm();
                        final int    number  = states[i].getManeuversNumber(maneuver.getName())  + 1;
                        final double cycleDV = states[i].getManeuversCycleDV(maneuver.getName()) + dv;
                        final double totalDV = states[i].getManeuversTotalDV(maneuver.getName()) + dv;

                        // update the state
                        states[i] = states[i].updateManeuverStats(maneuver.getName(),
                                                                  number, cycleDV, totalDV);

                        // monitor maneuver
                        SimpleMonitorable monitorable = maneuversMonitorables.get(maneuver.getName());
                        monitorable.setSampledValue(maneuverDate,
                                                    new double[] {
                                                      number, cycleDV, totalDV
                                                    });

                        // print the maneuver
                        maneuversOutput.println(maneuver.getDate()          + " " +
                                                maneuver.getName()          + " " +
                                                maneuver.getDeltaV().getX() + " " +
                                                maneuver.getDeltaV().getY() + " " +
                                                maneuver.getDeltaV().getZ() + " " +
                                                states[i].getName()         + " " +
                                                (maneuver.isReplanned() ? "replanned" : ""));
                    }
                }
            }
        }

    }

}
