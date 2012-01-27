/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.scenario;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
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

    /** Reference ground location. */
    private TopocentricFrame groundLocation;

    /** Mono-spacecraft monitorables. */
    private List<MonitorableMonoSKData> monitorablesMono;

    /** Duo-spacecrafts monitorables. */
    private List<MonitorableDuoSKData> monitorablesDuo;

    /** Maneuvers monitorables. */
    private final Map<String, SimpleMonitorable> maneuversMonitorables;

    /** Map for control laws values monitoring. */
    private Map<SKControl, SimpleMonitorable> controlsValues;

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
     * @param controlsValues map for control laws values monitoring
     * @param controlsViolations map for control laws violations monitoring
     * @param maneuversOutput maneuves output stream
     */
    public Scenario(final double outputStep,
                    final BodyShape earth, final CelestialBody sun,
                    final TopocentricFrame groundLocation,
                    final List<MonitorableMonoSKData> monitorablesMono,
                    final List<MonitorableDuoSKData> monitorablesDuo,
                    final Map<String, SimpleMonitorable> maneuversMonitorables,
                    final Map<SKControl, SimpleMonitorable> controlsValues,
                    final Map<SKControl, SimpleMonitorable> controlsViolations,
                    final PrintStream maneuversOutput) {
        this.components            = new ArrayList<ScenarioComponent>();
        this.outputstep            = outputStep;
        this.earth                 = earth;
        this.groundLocation        = groundLocation;
        this.monitorablesMono      = monitorablesMono;
        this.monitorablesDuo       = monitorablesDuo;
        this.maneuversMonitorables = maneuversMonitorables;
        this.controlsValues        = controlsValues;
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

    /** Set the cycle duration.
     * @param cycleDuration cycle duration
     */
    public void setCycleDuration(final double cycleDuration) {
        this.cycleDuration = cycleDuration;
    }

    /** {@inheritDoc}
     * <p>The scenario will be run in loops until the target date
     * is reached. At each iteration of the loop, the target date of
     * the iteration will be set according to the cycle duration set at
     * construction.</p>
     */
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException, SkatException {

        final AbsoluteDate cycleEnd = originals[0].getTargetCycleEnd();

        ScenarioState[] states = originals.clone();
        AbsoluteDate startDate = AbsoluteDate.FUTURE_INFINITY;
        for (final ScenarioState state : states) {
            if (state.getRealState().getDate().compareTo(startDate) < 0) {
                startDate = state.getRealState().getDate();
            }
        }

        while (cycleEnd.durationFrom(startDate) > 1.0) {

            // initial setting target date for iteration using cycle duration
            // (this will be updated by control loop)
            AbsoluteDate iterationTarget = startDate.shiftedBy(cycleDuration);
            if (iterationTarget.compareTo(cycleEnd) > 0) {
                iterationTarget = cycleEnd;
            }
            for (int i = 0; i < states.length; ++i) {
                states[i] = states[i].updateTargetCycleEnd(iterationTarget);
            }

            final TimeScale utc = TimeScalesFactory.getUTC();
            final Date now = Calendar.getInstance(TimeZone.getTimeZone("Etc/UTC")).getTime();
            System.out.println(new AbsoluteDate(now, utc).toString(utc) +
                               ": starting cycle " + states[0].getCyclesNumber() + " " +
                               startDate + " -> " + iterationTarget);

            // run all components of the scenario in order
            for (final ScenarioComponent component : components) {
                states = component.updateStates(states);
            }

            performMonitoring(states);

            // prepare next cycle
            Map<String, Integer> maneuverNb = new HashMap<String, Integer>();
            Map<String, Double>  maneuverDV = new HashMap<String, Double>();
            startDate = iterationTarget;
            for (int i = 0; i < states.length; ++i) {
                if (states[i].getRealState().getDate().compareTo(startDate) < 0) {
                    startDate = states[i].getRealState().getDate();
                }
                states[i] = states[i].updateCyclesNumber(states[i].getCyclesNumber() + 1);
                for (String name : states[i].getManeuversNames()) {
                    final int    manNb = states[i].getManeuversNumber(name);
                    final double manDV = states[i].getManeuversTotalDV(name);
                    states[i] = states[i].updateManeuverStats(name, manNb, 0.0, manDV);

                    final Integer nb = maneuverNb.get(name);
                    final Double  dv = maneuverDV.get(name);
                    if (nb == null || nb < manNb) {
                        maneuverNb.put(name, manNb);
                    }
                    if (dv == null || dv < manDV) {
                        maneuverDV.put(name, manDV);
                    }
                }
                states[i] = states[i].updateEstimatedState(null);
                states[i] = states[i].updateManeuvers(new ArrayList<ScheduledManeuver>());
            }
            
            // monitor maneuvers at cycle start
            for (final String name : states[0].getManeuversNames()) {
                SimpleMonitorable monitorable = maneuversMonitorables.get(name);
                monitorable.setSampledValue(states[0].getRealState().getDate(),
                                            new double[] {maneuverNb.get(name), 0.0, maneuverDV.get(name)});
            }

        }

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
        controls.addAll(controlsValues.keySet());
        controls.addAll(controlsViolations.keySet());
        for (int i = 0; i < states.length; ++i) {

            final BoundedPropagator propagator = states[i].getPerformedEphemeris();
            propagator.clearEventsDetectors();

            for (final SKControl controlLaw : controls) {
                controlLaw.setMonitoring(true);
                final List<ScheduledManeuver> maneuvers = new ArrayList<ScheduledManeuver>();
                if (states[i].getManeuvers() != null) {
                    for (final ScheduledManeuver m : states[i].getManeuvers()) {
                        if (m.getDate().compareTo(tMin) >= 0 && m.getDate().compareTo(tMax) < 0) {
                            maneuvers.add(m);
                        }
                    }
                }
                controlLaw.initializeRun(0, maneuvers.toArray(new ScheduledManeuver[maneuvers.size()]),
                                         propagator, new ArrayList<ScheduledManeuver>(), tMin, tMax);
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

            for (final SKControl controlLaw : controls) {
                controlLaw.setMonitoring(false);
            }

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
            for (final Map.Entry<SKControl, SimpleMonitorable> entry : controlsValues.entrySet()) {
                final double value = entry.getKey().getMonitoredValue(date);
                entry.getValue().setSampledValue(date, new double[] { value });
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
