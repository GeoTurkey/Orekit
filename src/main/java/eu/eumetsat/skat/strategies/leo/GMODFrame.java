/* Copyright 2002-2011 CS Communication & Systèmes
 * Licensed to CS Communication & Systèmes (CS) under one or more
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
package eu.eumetsat.skat.strategies.leo;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.MathUtils;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.frames.UpdatableFrame;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.DateComponents;
import org.orekit.time.TimeComponents;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;

/** Greenwich Mean Of Date Frame.
 * <p> This frame is linked to MoD frame and applies directly the mean sidereal
 * time according to IAU-82 model. It does <em>not</em> apply nutation. This frame
 * is useful only to define ground track grids which remain independent of short
 * periodic variations from nutation.</p>
 * <p> Its parent frame is the {@link MODFrame}.</p>
 * @author Luc Maisonobe
 */
//public class GMODFrame extends UpdatableFrame {
public class GMODFrame extends UpdatableFrame {

    /** Serializable UID. */
    private static final long serialVersionUID = 7206203752034399513L;

    /** Radians per second of time. */
    private static final double RADIANS_PER_SECOND = MathUtils.TWO_PI / Constants.JULIAN_DAY;

    /** Angular velocity of the Earth, in rad/s. */
    private static final double AVE = 7.292115146706979e-5;

    /** Reference date for IAU 1982 GMST-UT1 model. */
    private static final AbsoluteDate GMST_REFERENCE =
        new AbsoluteDate(DateComponents.J2000_EPOCH, TimeComponents.H12, TimeScalesFactory.getTAI());

    /** First coefficient of IAU 1982 GMST-UT1 model. */
    private static final double GMST_0 = 24110.54841;

    /** Second coefficient of IAU 1982 GMST-UT1 model. */
    private static final double GMST_1 = 8640184.812866;

    /** Third coefficient of IAU 1982 GMST-UT1 model. */
    private static final double GMST_2 = 0.093104;

    /** Fourth coefficient of IAU 1982 GMST-UT1 model. */
    private static final double GMST_3 = -6.2e-6;

    /** Cached date to avoid useless calculus. */
    private AbsoluteDate cachedDate;

    /** Simple constructor.
     * @exception OrekitException if EOP parameters are desired but cannot be read
     */
    public GMODFrame()
        throws OrekitException {

        super(FramesFactory.getMOD(false), null, "GMOD", false);

        // everything is in place, we can now synchronize the frame
        updateFrame(AbsoluteDate.J2000_EPOCH);

    }

    /** Compute mean sidereal time.
     * @param date date
     * @exception OrekitException if UTC-TAI correction cannot be loaded
     */
    public double getMeanSiderealTime(final AbsoluteDate date) throws OrekitException {

        // offset in julian centuries from J2000 epoch (UT1 scale)
        final double dtai = date.durationFrom(GMST_REFERENCE);
        final double dutc = TimeScalesFactory.getUTC().offsetFromTAI(date);

        final double tut1 = dtai + dutc;
        final double tt   = tut1 / Constants.JULIAN_CENTURY;

        // Seconds in the day, adjusted by 12 hours because the
        // UT1 is supplied as a Julian date beginning at noon.
        final double sd = (tut1 + Constants.JULIAN_DAY / 2.) % Constants.JULIAN_DAY;

        // compute Greenwich mean sidereal time, in radians
        return (((GMST_3 * tt + GMST_2) * tt + GMST_1) * tt + GMST_0 + sd) * RADIANS_PER_SECOND;

    }

    /** Update the frame to the given date.
     * <p>The update considers the earth rotation from IERS data.</p>
     * @param date new value of the date
     * @exception OrekitException if UTC-TAI correction cannot be loaded
     */
    protected void updateFrame(final AbsoluteDate date) throws OrekitException {

        if ((cachedDate == null) || !cachedDate.equals(date)) {

            // compute Greenwich mean sidereal time, in radians
            final double gmst = getMeanSiderealTime(date);

            // compute true angular rotation of Earth, in rad/s
            final Vector3D rotationRate = new Vector3D(AVE, Vector3D.PLUS_K);

            // set up the transform from parent TOD
            setTransform(new Transform(date, new Rotation(Vector3D.PLUS_K, -gmst), rotationRate));
            
            cachedDate = date;

        }
    }

}
