/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.orekit.time.AbsoluteDate;

import eu.eumetsat.skat.control.SKParameter;

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

    /** Indicator for in-plane maneuvers. */
    private final boolean inPlane;

    /** Indicator for maneuver relative to the previous one. */
    private final boolean relative;

    /** Thrust direction in spacecraft frame. */
    private final Vector3D direction;

    /** Specific impulse calibration curve. */
    private final double[][] isp;

    /** Tunable velocity increment. */
    private final SKParameter velocityIncrement;

    /** Nominal offset with respect to reference date. */
    private final double nominal;

    /** Reference date of the maneuver. */
    private AbsoluteDate reference;

    /** Current ISP. */
    private double currentIsp;

    /** Tunable date offset. */
    private final SKParameter dateOffset;

    /** Simple constructor.
     * @param name name of the maneuver
     * @param inPlane if true, the maneuver is considered to be in-plane
     * @param relative if true, the maneuver date is relative to the previous maneuver
     * @param direction thrust direction in spacecraft frame
     * @param isp engine specific impulse calibration curve (seconds in row 0, consumed mass in row 1)
     * @param minIncrement minimal allowed value for velocity increment
     * @param maxIncrement maximal allowed value for velocity increment
     * @param convergenceIncrement convergence threshold for velocity increment
     * @param nominal nominal offset with respect to reference date
     * @param minDateOffset offset for earliest allowed maneuver date
     * @param maxDateOffset offset for latest allowed maneuver date
     * @param convergenceDateOffset convergence threshold for sate offset
     */
    public TunableManeuver(final String name, final boolean inPlane,
                           final boolean relative, final Vector3D direction,
                           final double[][] isp,
                           final double minIncrement, final double maxIncrement,
                           final double convergenceIncrement,
                           final double nominal,
                           final double minDateOffset, final double maxDateOffset,
                           final double convergenceDateOffset) {
        this.inPlane      = inPlane;
        this.relative     = relative;
        this.direction    = direction.normalize();
        this.isp          = isp.clone();
        this.nominal      = nominal;
        velocityIncrement = new SKParameter(name + " (dV)", minIncrement, maxIncrement,
                                            convergenceIncrement,
                                            0.5 * (minIncrement + maxIncrement),
                                            FastMath.abs(maxIncrement - minIncrement) > 1.0e-6);
        dateOffset        = new SKParameter(name + " (date)", minDateOffset, maxDateOffset,
                                            convergenceDateOffset,
                                            0.5 * (minDateOffset + maxDateOffset),
                                            FastMath.abs(maxDateOffset - minDateOffset) > 1.0e-6);
    }

    /** Check if maneuver date is relative to the previous one.
     * @return true if maneuver date is relative to the previous one
     */
    public boolean isRelativeToPrevious() {
        return relative;
    }

    /** Set the reference date.
     * @param reference reference date
     */
    public void setReferenceDate(final AbsoluteDate date) {
        this.reference = date;
    }

    /** Set the reference consumed mass.
     * <p>
     * The reference consumed mass is used to interpolate the ISP to use from the calibration curve
     * </p>
     * @param consumedMass reference consumed mass
     */
    public void setReferenceConsumedMass(final double consumedMass) {

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
            }
        }

        // we have reached the end of the calibration curve,
        // we consider remaining ISP is constant
        currentIsp = isp[isp.length - 1][0];

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
     * @return maneuver corresponding to the current value of the parameters
     */
    public ScheduledManeuver getManeuver() {
        return new ScheduledManeuver(inPlane, getDate(),
                                     new Vector3D(velocityIncrement.getValue(), direction),
                                     currentIsp);
    }

    /** Get the maneuver date.
     * @return maneuver date
     */
    public AbsoluteDate getDate() {
        return reference.shiftedBy(nominal + dateOffset.getValue());
    }

}
