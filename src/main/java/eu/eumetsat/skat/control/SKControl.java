/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import java.util.List;

import org.orekit.errors.OrekitException;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;


/**
 * Interface representing a station-keeping control that should be achieved.
 * <p>
 * Station-keeping controls can be for example to ensure that spacecraft
 * remains within its allowed slot with as large margins as possible.
 * </p>
 * @see SKParameter
 * @author Luc Maisonobe
 */
public interface SKControl extends SKElement {

    /** Get the name of the control.
     * @return name of the control
     */
    String getName();

    /** Initialize one run of the control law.
     * <p>
     * This method is called at the start of each run with a set
     * of parameters. It can be used to reset some internal state
     * in the control law if needed.
     * </p>
     * @param iteration iteration number within the cycle
     * @param maneuvers maneuvers scheduled for this control law
     * @param propagator propagator for the cycle (it already takes
     * the maneuvers into account, but <em>none</em> of the {@link #getEventDetector()
     * event detector} and {@link #getStepHandler() step handler} if any)
     * @param fixedManeuvers list of maneuvers already fixed for the cycle
     * @param start start date of the propagation
     * @param end end date of the propagation
     * @param rollingCycles number of rolling cycles
     * @exception OrekitException if something weird occurs with the propagator
     */
    void initializeRun(int iteration, ScheduledManeuver[] maneuvers,
                       Propagator propagator, List<ScheduledManeuver> fixedManeuvers,
                       AbsoluteDate start, AbsoluteDate end, int rollingCycles)
        throws OrekitException;

    /** Check if the control limits have been exceeded.
     * @return a positive value if there are some margins, a negative value if
     * limits have been violated
     */
    double getMargins();

    /** Tune the maneuvers to improve control law fulfillment.
     * @param tunables tunable maneuvers
     * @return true if maneuver tuning has converged
     * @exception OrekitException if maneuvers cannot be tuned
     */
    boolean tuneManeuvers(TunableManeuver[] tunables) throws OrekitException;

    /** Get the name of the controlled spacecraft.
     * @return name of the controlled spacecraft.
     */
    String getControlledSpacecraftName();

    /** Get the index of the controlled spacecraft.
     * @return index of the controlled spacecraft.
     */
    int getControlledSpacecraftIndex();

    /** Get the name of the reference spacecraft.
     * @return name of the reference spacecraft
     * (null if control law does not depend on a reference spacecraft).
     */
    String getReferenceSpacecraftName();

    /** Get the index of the reference spacecraft.
     * @return index of the reference spacecraft
     * (-1 if control law does not depend on a reference spacecraft).
     */
    int getReferenceSpacecraftIndex();

    /** Get the event detector associated with this element.
     * @return event detector associated with this element (may be null)
     */
    EventDetector getEventDetector();

    /** Get the step handler associated with this element.
     * @return step handler associated with this element (may be null)
     */
    OrekitStepHandler getStepHandler();

}
