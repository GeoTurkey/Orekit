/* Copyright 2023 Luc Maisonobe
 * Licensed to CS GROUP (CS) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * CS licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.orekit.propagation.analytical.gnss.data;

/**
 * Container for data contained in a GPS/QZNSS civilian navigation message.
 * @author Luc Maisonobe
 * @since 12.0
 */
public class CivilianNavigationMessage extends AbstractNavigationMessage implements GNSSClockElements {

    /** Identifier for message type. */
    public static final String CNAV = "CNAV";

    /** Change rate in semi-major axis (m/s). */
    private double aDot;

    /** Change rate in Δn₀. */
    private double deltaN0Dot;

    /** Group Delay Differential (s). */
    private double tgd;

    /** The user SV accuracy (m). */
    private double svAccuracy;

    /** Satellite health status. */
    private double svHealth;

    /** Inter Signal Delay for L1 C/A. */
    private double iscL1CA;

    /** Inter Signal Delay for L2 C. */
    private double iscL2C;

    /** Inter Signal Delay for L5I. */
    private double iscL5I5;

    /** Inter Signal Delay for L5Q. */
    private double iscL5Q5;

    /** Term 0 of Non-Elevation-Dependent User Range Accuracy. */
    private double uraiNed0;

    /** Term 1 of Non-Elevation-Dependent User Range Accuracy. */
    private double uraiNed1;

    /** Term 2 of Non-Elevation-Dependent User Range Accuracy. */
    private double uraiNed2;

    /**
     * Constructor.
     * @param mu Earth's universal gravitational parameter
     * @param angularVelocity mean angular velocity of the Earth for the GNSS model
     * @param weekNumber number of weeks in the GNSS cycle
     */
    protected CivilianNavigationMessage(final double mu,
                                        final double angularVelocity,
                                        final int weekNumber) {
        super(mu, angularVelocity, weekNumber);
    }

    /**
     * Getter for the change rate in semi-major axis.
     * @return the change rate in semi-major axis
     */
    public double getADot() {
        return aDot;
    }

    /**
     * Setter for the change rate in semi-major axis.
     * @param value the change rate in semi-major axis
     */
    public void setADot(final double value) {
        this.aDot = value;
    }

    /**
     * Getter for change rate in Δn₀.
     * @return change rate in Δn₀
     */
    public double getDeltaN0Dot() {
        return deltaN0Dot;
    }

    /**
     * Setter for change rate in Δn₀.
     * @param deltaN0Dot change rate in Δn₀
     */
    public void setDeltaN0Dot(final double deltaN0Dot) {
        this.deltaN0Dot = deltaN0Dot;
    }

    /**
     * Getter for the Group Delay Differential (s).
     * @return the Group Delay Differential in seconds
     */
    public double getTGD() {
        return tgd;
    }

    /**
     * Setter for the Group Delay Differential (s).
     * @param time the group delay differential to set
     */
    public void setTGD(final double time) {
        this.tgd = time;
    }

    /**
     * Getter for the user SV accuray (meters).
     * @return the user SV accuracy
     */
    public double getSvAccuracy() {
        return svAccuracy;
    }

    /**
     * Setter for the user SV accuracy.
     * @param svAccuracy the value to set
     */
    public void setSvAccuracy(final double svAccuracy) {
        this.svAccuracy = svAccuracy;
    }

    /**
     * Getter for the satellite health status.
     * @return the satellite health status
     */
    public double getSvHealth() {
        return svHealth;
    }

    /**
     * Setter for the satellite health status.
     * @param svHealth the value to set
     */
    public void setSvHealth(final double svHealth) {
        this.svHealth = svHealth;
    }

    /**
     * Getter for inter Signal Delay for L1 C/A.
     * @return inter signal delay
     */
    public double getIscL1CA() {
        return iscL1CA;
    }

    /**
     * Setter for inter Signal Delay for L1 C/A.
     * @param delay delay to set
     */
    public void setIscL1CA(final double delay) {
        this.iscL1CA = delay;
    }

    /**
     * Getter for inter Signal Delay for L2 C.
     * @return inter signal delay
     */
    public double getIscL2C() {
        return iscL2C;
    }

    /**
     * Setter for inter Signal Delay for L2 C.
     * @param delay delay to set
     */
    public void setIscL2C(final double delay) {
        this.iscL2C = delay;
    }

    /**
     * Getter for inter Signal Delay for L5I.
     * @return inter signal delay
     */
    public double getIscL5I5() {
        return iscL5I5;
    }

    /**
     * Setter for inter Signal Delay for L5I.
     * @param delay delay to set
     */
    public void setIscL5I5(final double delay) {
        this.iscL5I5 = delay;
    }

    /**
     * Getter for inter Signal Delay for L5Q.
     * @return inter signal delay
     */
    public double getIscL5Q5() {
        return iscL5Q5;
    }

    /**
     * Setter for inter Signal Delay for L5Q.
     * @param delay delay to set
     */
    public void setIscL5Q5(final double delay) {
        this.iscL5Q5 = delay;
    }

    /**
     * Getter for term 0 of Non-Elevation-Dependent User Range Accuracy.
     * @return term 0 of Non-Elevation-Dependent User Range Accuracy
     */
    public double getUraiNed0() {
        return uraiNed0;
    }

    /**
     * Setter for term 0 of Non-Elevation-Dependent User Range Accuracy.
     * @param uraiNed0 term 0 of Non-Elevation-Dependent User Range Accuracy
     */
    public void setUraiNed0(final double uraiNed0) {
        this.uraiNed0 = uraiNed0;
    }

    /**
     * Getter for term 1 of Non-Elevation-Dependent User Range Accuracy.
     * @return term 1 of Non-Elevation-Dependent User Range Accuracy
     */
    public double getUraiNed1() {
        return uraiNed1;
    }

    /**
     * Setter for term 1 of Non-Elevation-Dependent User Range Accuracy.
     * @param uraiNed1 term 1 of Non-Elevation-Dependent User Range Accuracy
     */
    public void setUraiNed1(final double uraiNed1) {
        this.uraiNed1 = uraiNed1;
    }

    /**
     * Getter for term 2 of Non-Elevation-Dependent User Range Accuracy.
     * @return term 2 of Non-Elevation-Dependent User Range Accuracy
     */
    public double getUraiNed2() {
        return uraiNed2;
    }

    /**
     * Setter for term 2 of Non-Elevation-Dependent User Range Accuracy.
     * @param uraiNed2 term 2 of Non-Elevation-Dependent User Range Accuracy
     */
    public void setUraiNed2(final double uraiNed2) {
        this.uraiNed2 = uraiNed2;
    }

}
