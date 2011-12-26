/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.orekit.propagation.analytical.ManeuverAdapterPropagator;
import org.orekit.time.AbsoluteDate;

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

    /** Thrust direction in spacecraft frame. */
    private final Vector3D direction;

    /** Engine thrust calibration curve. */
    private final double[][] thrust;

    /** Specific impulse calibration curve. */
    private final double[][] isp;

    /** Lower bound for velocity increment. */
    private final double dVInf;

    /** Upper bound for velocity increment. */
    private final double dVSup;

    /** Convergence threshold for velocity increment. */
    private final double dVConvergence;

    /** Convergence threshold for date. */
    private final double dTConvergence;

    /** Elimination threshold. */
    private final double elimination;

    /** Current thrust. */
    private double currentThrust;

    /** Current ISP. */
    private double currentIsp;

    /** Simple constructor.
     * @param name name of the maneuver
     * @param direction thrust direction in spacecraft frame
     * @param thrust engine thrust calibration curve (Newtons in row 0, consumed mass in row 1)
     * @param isp engine specific impulse calibration curve (seconds in row 0, consumed mass in row 1)
     * @param elimination elimination threshold
     * @param dVInf lower bound for velocity increment
     * @param dVSup upper bound for velocity increment
     * @param dVConvergence convergence threshold for velocity increment
     * @param dTConvergence convergence threshold for date offset
     * @exception SkatException if calibration curves are not ordered
     */
    public TunableManeuver(final String name, final Vector3D direction,
                           final double[][] thrust, final double[][] isp, final double elimination,
                           final double dVInf, final double dVSup,
                           final double dVConvergence, final double dTConvergence)
        throws SkatException {
        this.name                  = name;
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

        this.elimination   = elimination;
        this.dVInf         = dVInf;
        this.dVSup         = dVSup;
        this.dVConvergence = dVConvergence;
        this.dTConvergence = dTConvergence;

    }

    /** Get the name of the maneuver.
     * @return name of the maneuver
     */
    public String getName() {
        return name;
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
    public double getDTConvergence() {
        return dTConvergence;
    }

    /** Get the convergence threshold for velocity increment.
     * @return convergence threshold for velocity increment
     */
    public double getDVConvergence() {
        return dVConvergence;
    }

    /** Get the lower bound for velocity increment.
     * @return lower bound for velocity increment
     */
    public double getDVInf() {
        return dVInf;
    }

    /** Get the upper bound for velocity increment.
     * @return upper bound for velocity increment
     */
    public double getDVSup() {
        return dVSup;
    }

    /** Get the maneuver corresponding to the current value of the parameters.
     * @param date maneuver date
     * @param deltaV velocity increment along thrust direction
     * @param trajectory to which this maneuver belongs
     * @return maneuver corresponding to the current value of the parameters
     */
    public ScheduledManeuver buildManeuver(final AbsoluteDate date, final double deltaV,
                                           final ManeuverAdapterPropagator trajectory) {
        return new ScheduledManeuver(this, date,
                                     new Vector3D(deltaV, direction), currentThrust,
                                     currentIsp, trajectory, false);
    }

}
