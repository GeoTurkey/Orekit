/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;
import java.util.Arrays;

import org.orekit.time.AbsoluteDate;


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

    /** Lower bound for velocity increment, calibration curve */
    private final double[][] dVInf;

    /** Upper bound for velocity increment, calibration curve. */
    private final double[][] dVSup;

    /** Current lower bound for velocity increment. */
    private double currentDVInf;

    /** Current upper bound for velocity increment. */
    private double currentDVSup;

    /** Convergence threshold for velocity increment. */
    private final double dVConvergence;

    /** Convergence threshold for date. */
    private final double dTConvergence;

    /** Elimination threshold. */
    private final double elimination;

    /** End date of the simulation. */
    private AbsoluteDate endDate;

    /** Current thrust. */
    private double currentThrust;

    /** Current ISP. */
    private double currentIsp;

    /** Flag for the necessity of a slew prior to the maneuver */
	private boolean isPreviousSlew;

    /** Slew prior to the maneuver: fixed delta-V */
	private Vector3D previousSlewDeltaV;

    /** Slew prior to the maneuver: fixed mass consumption */
	private double previousSlewDeltaMass;

    /** Slew prior to the maneuver: fixed delay between slew and maneuver */
	private double previousSlewDelay;

    /** Flag for the necessity of a slew after to the maneuver */
	private boolean isFollowingSlew;

    /** Slew after the maneuver: fixed delta-V */
	private Vector3D followingSlewDeltaV;

    /** Slew after to the maneuver: fixed mass consumption */
	private double followingSlewDeltaMass;

	/** Slew after to the maneuver: fixed delay between maneuver and slew */
	private double followingSlewDelay;

	/** Simulation start date */
	private AbsoluteDate beginningDate;

    /** Simple constructor.
     * @param name name of the maneuver
     * @param direction thrust direction in spacecraft frame
     * @param thrust engine thrust calibration curve (Newtons in row 0, consumed mass in row 1)
     * @param isp engine specific impulse calibration curve (seconds in row 0, consumed mass in row 1)
     * @param elimination elimination threshold
     * @param dVInf lower bound for velocity increment
     * @param dVSup upper bound for velocity increment
     * @param beginningDate simulation beginning date
     * @param endDate simulation ending date
     * @param dVConvergence convergence threshold for velocity increment
     * @param dTConvergence convergence threshold for date offset
     * @exception SkatException if calibration curves are not ordered
     */
    public TunableManeuver(final String name, final Vector3D direction,
                           final double[][] thrust, final double[][] isp, final double elimination,
                           final double[][] dVInf,  final double[][] dVSup,
                           final double dVConvergence, final double dTConvergence,
                           final AbsoluteDate beginningDate, final AbsoluteDate endDate, 
                           final boolean isPreviousSlew,  final Vector3D previousSlewDeltaV,  final double previousSlewDeltaMass,  final double previousSlewDelay,
                           final boolean isFollowingSlew, final Vector3D followingSlewDeltaV, final double followingSlewDeltaMass, final double followingSlewDelay)
        throws SkatException {
        this.name                  = name;
        this.direction             = direction.normalize();
        this.thrust                = thrust.clone();
        this.beginningDate         = beginningDate;
        this.endDate               = endDate;
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
        this.dVInf = dVInf.clone();
        for (int i = 1; i < dVInf.length; ++i) {
            if (dVInf[i - 1][1] > dVInf[i][1]) {
                throw new SkatException(SkatMessages.NON_INCREASING_MASSES_IN_DELTAV_INF_CALIBRATION_CURVE,
                		                dVInf[i - 1][1], dVInf[i][1]);
            }
        }
        this.dVSup  = dVSup.clone();
        for (int i = 1; i < dVSup.length; ++i) {
            if (dVSup[i - 1][1] > dVSup[i][1]) {
                throw new SkatException(SkatMessages.NON_INCREASING_MASSES_IN_DELTAV_SUP_CALIBRATION_CURVE,
                		                dVSup[i - 1][1], dVSup[i][1]);
            }
        }

        this.elimination    = elimination;
        this.dVConvergence  = dVConvergence;
        this.dTConvergence  = dTConvergence;
        this.isPreviousSlew = isPreviousSlew;
        this.previousSlewDeltaV     = previousSlewDeltaV;
        this.previousSlewDeltaMass  = previousSlewDeltaMass;
        this.previousSlewDelay      = previousSlewDelay;
        this.isFollowingSlew        = isFollowingSlew;
        this.followingSlewDeltaV    = followingSlewDeltaV;
        this.followingSlewDeltaMass = followingSlewDeltaMass;
        this.followingSlewDelay     = followingSlewDelay;

    }

    /** Constructor for attitude variation (copy a model and change the thrust direction)
     * @param model model to be copied
     * @param direction new thrust direction
     */
    public TunableManeuver(final TunableManeuver model, final Vector3D direction)
    {
    	this.direction = direction.normalize();
    	
        this.name          = model.name;
        this.thrust        = model.thrust.clone();
        this.isp           = model.isp.clone();
        this.endDate       = model.endDate;
        this.elimination   = model.elimination;
        this.dVInf         = model.dVInf.clone();
        this.dVSup         = model.dVSup.clone();
        this.currentDVInf  = model.currentDVInf;
        this.currentDVSup  = model.currentDVSup;
        this.dVConvergence = model.dVConvergence;
        this.dTConvergence = model.dTConvergence;
        this.currentThrust = model.currentThrust;;
        this.currentIsp    = model.currentIsp;;

}

    /** Get the name of the maneuver.
     * @return name of the maneuver
     */
    public String getName() {
        return name;
    }

    /** Set the reference consumed mass.
     * <p>
     * The reference consumed mass is used to interpolate the thrust, ISP and extreme delta-V
     * to use from the calibration curves.
     * </p>
     * @param consumedMass reference consumed mass
     */
    public void setReferenceConsumedMass(final double consumedMass) {
        updateThrust(consumedMass);
        updateISP(consumedMass);
        updateDVInf(consumedMass);
        updateDVSup(consumedMass);
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

        if (consumedMass >= thrust[thrust.length-1][1]) {
            // mass is greater than last curve point,
            // we are in a regulated phase, thrust is constant
            currentThrust = thrust[thrust.length-1][0];
            return;
        }

        for (int i = thrust.length-1; i > 0 ; --i) {
            if (consumedMass >= thrust[i-1][1]) {
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

        // otherwise, we assign first point
        currentThrust = thrust[0][0];

    }

    /** Update the current lower delta-V bound.
     * @param consumedMass reference consumed mass
     */
    public void updateDVInf(final double consumedMass) {

        if (consumedMass >= dVInf[dVInf.length-1][1]) {
            // mass is greater than last curve point,
            // we are in a regulated phase, extreme delta-V is constant
            currentDVInf = dVInf[dVInf.length-1][0];
            return;
        }

        for (int i = dVInf.length-1; i > 0 ; --i) {
            if (consumedMass >= dVInf[i-1][1]) {
                // we are in an interval between two curve points
                // we are in blow-down mode, extreme delta-V evolves linearly
                final double dVInf0 = dVInf[i - 1][0];
                final double mass0  = dVInf[i - 1][1];
                final double dVInf1 = dVInf[i][0];
                final double mass1  = dVInf[i][1];
                currentDVInf = (dVInf0 * (consumedMass - mass1) + dVInf1 * (mass0 - consumedMass)) /
                                (mass0 - mass1);
                return;
            }
        }

        // otherwise, we assign first point
        currentDVInf = dVInf[0][0];

    }

    /** Update the current upper delta-V bound.
     * @param consumedMass reference consumed mass
     */
    public void updateDVSup(final double consumedMass) {

        if (consumedMass >= dVSup[dVSup.length-1][1]) {
            // mass is greater than last curve point,
            // we are in a regulated phase, extreme delta-V is constant
            currentDVSup = dVSup[dVSup.length-1][0];
            return;
        }

        for (int i = dVSup.length-1; i > 0 ; --i) {
            if (consumedMass >= dVSup[i-1][1]) {
                // we are in an interval between two curve points
                // we are in blow-down mode, extreme delta-V evolves linearly
                final double dVSup0 = dVSup[i - 1][0];
                final double mass0  = dVSup[i - 1][1];
                final double dVSup1 = dVSup[i][0];
                final double mass1  = dVSup[i][1];
                currentDVSup = (dVSup0 * (consumedMass - mass1) + dVSup1 * (mass0 - consumedMass)) /
                                (mass0 - mass1);
                return;
            }
        }

        // otherwise, we assign first point
        currentDVSup = dVSup[0][0];

    }

    /** Update the current specific impulse.
     * @param consumedMass reference consumed mass
     */
    public void updateISP(final double consumedMass) {

        if (consumedMass >= isp[isp.length-1][1]) {
            // mass is greater than last curve point,
            // we are in a regulated phase, ISP is constant
            currentIsp = isp[isp.length-1][0];
            return;
        }

        for (int i = isp.length-1; i > 0 ; --i) {
            if (consumedMass >= isp[i-1][1]) {
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

        // otherwise, we assign first point
        currentIsp = isp[0][0];

    }

    /** Get the current value of the thrust.
     * @return current value of the thrust
     * @see #updateThrust(double)
     */
    public double getCurrentThrust() {
        return currentThrust;
    }

    /** Get the current value of the specific impulse.
     * @return current value of the specific impulse
     * @see #updateISP(double)
     */
    public double getCurrentISP() {
        return currentIsp;
    }

    /** Get the current value of the lower bound for velocity increment.
     * @return current value of the lower bound for velocity increment
     * @see #updateDVInf(double)
     */
    public double getCurrentDVInf() {
        return currentDVInf;
    }

    /** Get the current value of the upper bound for velocity increment.
     * @return current value of the upper bound for velocity increment
     * @see #updateDVSup(double)
     */
    public double getCurrentDVSup() {
        return currentDVSup;
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

    /** Get end date of the simulation.
    * @return end date of the simulation
    */
   public AbsoluteDate getEndDate() {
       return endDate;
   }

    /** Get the convergence threshold for velocity increment.
     * @return convergence threshold for velocity increment
     */
    public double getDVConvergence() {
        return dVConvergence;
    }

    /** Get the flag for the presence of a previous mass-consuming slew.
     * @return true if there is a previous slew, false otherwise
     */
    public boolean isPreviousSlew() {
            return isPreviousSlew;
    }

    public Vector3D getPreviousSlewDeltaV() {
            return previousSlewDeltaV;
    }

    public double getPreviousSlewDeltaMass() {
            return previousSlewDeltaMass;
    }

    public double getPreviousSlewDelay() {
            return previousSlewDelay;
    }

    public boolean isFollowingSlew() {
            return isFollowingSlew;
    }

    public Vector3D getFollowingSlewDeltaV() {
            return followingSlewDeltaV;
    }

    public double getFollowingSlewDeltaMass() {
            return followingSlewDeltaMass;
    }

    public double getFollowingSlewDelay() {
            return followingSlewDelay;
    }

    public AbsoluteDate getBeginningDate() {
            return beginningDate;
    }
    
    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final TunableManeuver other = (TunableManeuver) obj;
        
        final boolean dirEqual = Vector3D.angle(this.direction,other.direction) < 1e-6;
        final boolean beginningDateEqual = (this.beginningDate == null && other.beginningDate == null)  || this.beginningDate.equals(other.beginningDate);
        final boolean endDateEqual       = (this.endDate == null && other.endDate == null)  || this.endDate.equals(other.endDate);
        final boolean previousSlewEqual;
        final boolean followingSlewEqual;
        
        if (!this.isPreviousSlew && !this.isPreviousSlew) {
            // this.previousSlewDeltaV and other.previousSlewDeltaV are both null
            previousSlewEqual = true;
        } else if (this.isPreviousSlew && this.isPreviousSlew) {
            previousSlewEqual = this.previousSlewDeltaV.equals(other.previousSlewDeltaV);
        } else {
            previousSlewEqual = false;
        }
        
        if (!this.isFollowingSlew && !this.isFollowingSlew) {
            // this.followingSlewDeltaV and other.followingSlewDeltaV are both null
            followingSlewEqual = true;
        } else if (this.isFollowingSlew && this.isFollowingSlew) {
            followingSlewEqual = this.followingSlewDeltaV.equals(other.followingSlewDeltaV);
        } else {
            followingSlewEqual = false;
        }
        
        
        if ( dirEqual && this.name.equals(other.name) && Arrays.deepEquals(this.thrust,other.thrust)
                && Arrays.deepEquals(this.isp,other.isp) && this.elimination == other.elimination
                && Arrays.deepEquals(this.dVInf,other.dVInf) && Arrays.deepEquals(this.dVSup,other.dVSup)
                && this.dVConvergence == other.dVConvergence && this.dTConvergence == other.dTConvergence
                && beginningDateEqual && endDateEqual
                && this.isPreviousSlew == other.isPreviousSlew && previousSlewEqual
                && this.previousSlewDeltaMass == other.previousSlewDeltaMass && this.previousSlewDelay == other.previousSlewDelay
                && this.isFollowingSlew == other.isFollowingSlew && followingSlewEqual
                && this.followingSlewDeltaMass == other.followingSlewDeltaMass && this.followingSlewDelay == other.followingSlewDelay) {
            return true;
        }
        
        return false;
    }
}