/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.orekit.propagation.analytical.ManeuverAdapterPropagator;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.SKParameter;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;

/**
 * This class represents an impulse maneuver with free parameters for date
 * and velocity increment.
 * <p>
 * The specific impulse of the maneuver involves linearly between an ISP
 * calibration curve according to consumed mass.
 * </p>
 * @author Luc Maisonobe
 */
public class TunableManeuver {

    /** Name of the maneuver.*/
    private final String name;

    /** Indicator for in-plane maneuvers. */
    private final boolean inPlane;

    /** Reference maneuver for date (may be null). */
    private final TunableManeuver dateReferenceManeuver;

    /** Reference maneuver for dv (may be null). */
    private final TunableManeuver dVReferenceManeuver;

    /** Thrust direction in spacecraft frame. */
    private final Vector3D direction;

    /** Engine thrust calibration curve. */
    private final double[][] thrust;

    /** Specific impulse calibration curve. */
    private final double[][] isp;

    /** Elimination threshold. */
    private final double elimination;

    /** Nominal offset with respect to reference dV. */
    private final double dVNominal;

    /** Tunable velocity increment. */
    private final SKParameter velocityIncrement;

    /** Nominal offset with respect to reference date. */
    private final double dtNominal;

    /** Cycle start date. */
    private AbsoluteDate cycleStart;

    /** Current thrust. */
    private double currentThrust;

    /** Current ISP. */
    private double currentIsp;

    /** Tunable date offset. */
    private final SKParameter dateOffset;

    /** Simple constructor.
     * @param name name of the maneuver
     * @param inPlane if true, the maneuver is considered to be in-plane
     * @param dateReferenceManeuver reference maneuver for date (may be null)
     * @param dVReferenceManeuver reference maneuver for date (may be null)
     * @param direction thrust direction in spacecraft frame
     * @param thrust engine thrust calibration curve (Newtons in row 0, consumed mass in row 1)
     * @param isp engine specific impulse calibration curve (seconds in row 0, consumed mass in row 1)
     * @param elimination elimination threshold
     * @param dVNominal nominal offset with respect to reference dV
     * @param minIncrement minimal allowed value for velocity increment
     * @param maxIncrement maximal allowed value for velocity increment
     * @param convergenceIncrement convergence threshold for velocity increment
     * @param dtNominal nominal offset with respect to reference date
     * @param minDateOffset offset for earliest allowed maneuver date
     * @param maxDateOffset offset for latest allowed maneuver date
     * @param convergenceDateOffset convergence threshold for date offset
     * @exception SkatException if calibration curves are not ordered
     */
    public TunableManeuver(final String name, final boolean inPlane,
                           final TunableManeuver dateReferenceManeuver,
                           final TunableManeuver dVReferenceManeuver,
                           final Vector3D direction,
                           final double[][] thrust, final double[][] isp,
                           final double elimination,
                           final double dVNominal, final double minIncrement, final double maxIncrement,
                           final double convergenceIncrement,
                           final double dtNominal, final double minDateOffset, final double maxDateOffset,
                           final double convergenceDateOffset)
        throws SkatException {
        this.name                  = name;
        this.inPlane               = inPlane;
        this.dateReferenceManeuver = dateReferenceManeuver;
        this.dVReferenceManeuver   = dVReferenceManeuver;
        this.direction             = direction.normalize();
        this.thrust                = thrust.clone();
        for (int i = 1; i < thrust.length; ++i) {
            if (thrust[i - 1][1] > thrust[i][1]) {
                throw new SkatException(SkatMessages.NON_INCREASING_MASSES_IN_THRUST_CALIBRATION_CURVE,
                                        thrust[i - 1][1], thrust[i][1]);
            }
        }
        this.isp          = isp.clone();
        for (int i = 1; i < isp.length; ++i) {
            if (isp[i - 1][1] > isp[i][1]) {
                throw new SkatException(SkatMessages.NON_INCREASING_MASSES_IN_ISP_CALIBRATION_CURVE,
                                        isp[i - 1][1], isp[i][1]);
            }
        }
        this.elimination  = elimination;
        this.dVNominal    = dVNominal;
        this.dtNominal    = dtNominal;
        velocityIncrement = new SKParameter(name + " (dV)", minIncrement, maxIncrement,
                                            convergenceIncrement,
                                            0.5 * (minIncrement + maxIncrement),
                                            FastMath.abs(maxIncrement - minIncrement) > 1.0e-6);
        dateOffset        = new SKParameter(name + " (date)", minDateOffset, maxDateOffset,
                                            convergenceDateOffset,
                                            0.5 * (minDateOffset + maxDateOffset),
                                            FastMath.abs(maxDateOffset - minDateOffset) > 1.0e-6);
    }

    /** Get the name of the maneuver.
     * @return name of the maneuver
     */
    public String getName() {
        return name;
    }

    /** Set the cycle start date.
     * @param cycleStart cycle start date
     */
    public void setCycleStartDate(final AbsoluteDate cycleStart) {
        this.cycleStart = cycleStart;
    }

    /** Set the reference consumed mass.
     * <p>
     * The reference consumed mass is used to interpolate the thrust and ISP
     * to use from the calibration curves.
     * </p>
     * @param consumedMass reference consumed mass
     */
    public void setReferenceConsumedMass(final double consumedMass) {
        updateThrust(consumedMass);
        updateISP(consumedMass);
    }

    /** Get the thrust direction in spacecraft frame.
     * @return thrust direction in spacecraft frame
     */
    public Vector3D getDirection() {
        return direction;
    }

    /** Update the current thrust.
     * @param consumedMass reference consumed mass
     */
    public void updateThrust(final double consumedMass) {

        if (consumedMass >= thrust[0][1]) {
            // mass is greater than first curve point,
            // we are in a regulated phase, thrust is constant
            currentThrust = thrust[0][0];
            return;
        }

        for (int i = 1; i < thrust.length; ++i) {
            if (consumedMass >= thrust[i][1]) {
                // we are in an interval between two curve points
                // we are in blow-down mode, thrust evolves linearly
                final double thrust0  = thrust[i - 1][0];
                final double mass0    = thrust[i - 1][1];
                final double thrust1  = thrust[i][0];
                final double mass1    = thrust[i][1];
                currentThrust = (thrust0 * (consumedMass - mass1) + thrust1 * (mass0 - consumedMass)) /
                                (mass0 - mass1);
                return;
            }
        }

        // we have reached the end of the calibration curve,
        // we consider remaining thrust is constant
        currentThrust = thrust[thrust.length - 1][0];

    }

    /** Update the current specific impulse.
     * @param consumedMass reference consumed mass
     */
    public void updateISP(final double consumedMass) {

        if (consumedMass >= isp[0][1]) {
            // mass is greater than first curve point,
            // we are in a regulated phase, ISP is constant
            currentIsp = isp[0][0];
            return;
        }

        for (int i = 1; i < isp.length; ++i) {
            if (consumedMass >= isp[i][1]) {
                // we are in an interval between two curve points
                // we are in blow-down mode, ISP evolves linearly
                final double isp0  = isp[i - 1][0];
                final double mass0 = isp[i - 1][1];
                final double isp1  = isp[i][0];
                final double mass1 = isp[i][1];
                currentIsp = (isp0 * (consumedMass - mass1) + isp1 * (mass0 - consumedMass)) /
                             (mass0 - mass1);
                return;
            }
        }

        // we have reached the end of the calibration curve,
        // we consider remaining ISP is constant
        currentIsp = isp[isp.length - 1][0];

    }

    /** Get the elimination threshold.
     * @return elimination threshold
     */
    public double getEliminationThreshold() {
        return elimination;
    }

    /** Get the convergence threshold for date offset.
     * @return convergence threshold for date offset
     */
    public double getDateConvergence() {
        return dateOffset.getConvergence();
    }

    /** Get the minimum allowed value for date offset.
     * @return minimum allowed value for date offset
     */
    public double getDateOffsetMin() {
        return dateOffset.getMin();
    }

    /** Get the maximum allowed value for date offset.
     * @return maximum allowed value for date offset
     */
    public double getDateOffsetMax() {
        return dateOffset.getMax();
    }

    /** Get the convergence threshold for velocity increment.
     * @return convergence threshold for velocity increment
     */
    public double getDVConvergence() {
        return velocityIncrement.getConvergence();
    }

    /** Get the minimum allowed value for velocity increment.
     * @return minimum allowed value for velocity increment
     */
    public double getDVMin() {
        return velocityIncrement.getMin();
    }

    /** Get the maximum allowed value for velocity increment.
     * @return maximum allowed value for velocity increment
     */
    public double getDVMax() {
        return velocityIncrement.getMax();
    }

    /** Get the maneuver parameters.
     * @return list of maneuver parameters
     */
    public List<SKParameter> getParameters() {
        final List<SKParameter> list = new ArrayList<SKParameter>(2);
        list.add(velocityIncrement);
        list.add(dateOffset);
        return list;
    }

    /** Get the maneuver corresponding to the current value of the parameters.
     * @param date maneuver date
     * @param deltaV velocity increment along thrust direction
     * @param trajectory to which this maneuver belongs
     * @return maneuver corresponding to the current value of the parameters
     */
    public ScheduledManeuver buildManeuver(final AbsoluteDate date, final double deltaV,
                                           final ManeuverAdapterPropagator trajectory) {
        return new ScheduledManeuver(this, inPlane, date, new Vector3D(deltaV, direction),
                                     currentThrust, currentIsp, trajectory, false);
    }

    /** Get the maneuver velocity increment.
     * @return maneuver velocity increment
     */
    public double getDV() {
        final double referenceDV = dVNominal +
                                   ((dVReferenceManeuver == null) ? 0 : dVReferenceManeuver.getDV());
        return referenceDV + velocityIncrement.getValue();
    }

    /** Get the maneuver date.
     * @return maneuver date
     */
    public AbsoluteDate getDate() {
        final AbsoluteDate reference = (dateReferenceManeuver == null) ?
                                       cycleStart : dateReferenceManeuver.getDate();
        return reference.shiftedBy(dtNominal + dateOffset.getValue());
    }

    /** Get earliest date offset from cycle start.
     * @return earliest date offset from cycle start
     */
    public double getEarliestDateOffset() {
        return dateOffset.getMin() + dtNominal +
               ((dateReferenceManeuver == null) ? 0 : dateReferenceManeuver.getEarliestDateOffset());
    }

    /** Get latest date offset from cycle start.
     * @return latest date offset from cycle start
     */
    public double getLatestDateOffset() {
        return dateOffset.getMax() + dtNominal +
               ((dateReferenceManeuver == null) ? 0 : dateReferenceManeuver.getLatestDateOffset());
    }

}
