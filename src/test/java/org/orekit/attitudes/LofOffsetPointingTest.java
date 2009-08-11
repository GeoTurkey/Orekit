/* Copyright 2002-2008 CS Communication & Systèmes
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
package org.orekit.attitudes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import org.apache.commons.math.geometry.Rotation;
import org.apache.commons.math.geometry.RotationOrder;
import org.apache.commons.math.geometry.Vector3D;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.orekit.Utils;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataProvidersManager;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CircularOrbit;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.DateComponents;
import org.orekit.time.TimeComponents;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.PVCoordinates;


public class LofOffsetPointingTest {

    // Computation date 
    private AbsoluteDate date;
    
    // Body mu 
    private double mu;

    // Reference frame = ITRF 2005C 
    private Frame frameItrf2005;
        
    // Earth shape
    OneAxisEllipsoid earthSpheric;

    /** Test if both constructors are equivalent
     */
    @Test
    public void testLof() throws OrekitException {

        //  Satellite position
        final CircularOrbit circ =
            new CircularOrbit(7178000.0, 0.5e-4, -0.5e-4, Math.toRadians(0.), Math.toRadians(270.),
                                   Math.toRadians(5.300), CircularOrbit.MEAN_LONGITUDE_ARGUMENT, 
                                   FramesFactory.getEME2000(), date, mu);
        final PVCoordinates pvSatEME2000 = circ.getPVCoordinates();

        // Create lof aligned law
        //************************
        final LofOffset lofLaw = LofOffset.LOF_ALIGNED;
        final LofOffsetPointing lofPointing = new LofOffsetPointing(earthSpheric, lofLaw, Vector3D.PLUS_K);
        final Rotation lofRot = lofPointing.getState(date, pvSatEME2000, FramesFactory.getEME2000()).getRotation();
 
        // Compare to body center pointing law
        //*************************************
        final BodyCenterPointing centerLaw = new BodyCenterPointing(earthSpheric.getBodyFrame());
        final Rotation centerRot = centerLaw.getState(date, pvSatEME2000, FramesFactory.getEME2000()).getRotation();
        final double angleBodyCenter = centerRot.applyInverseTo(lofRot).getAngle();
        assertEquals(0., angleBodyCenter, Utils.epsilonAngle);

        // Compare to nadir pointing law
        //*******************************
        final NadirPointing nadirLaw = new NadirPointing(earthSpheric);
        final Rotation nadirRot = nadirLaw.getState(date, pvSatEME2000, FramesFactory.getEME2000()).getRotation();
        final double angleNadir = nadirRot.applyInverseTo(lofRot).getAngle();
        assertEquals(0., angleNadir, Utils.epsilonAngle);

    } 

    @Test
    public void testMiss() {
        final CircularOrbit circ =
            new CircularOrbit(7178000.0, 0.5e-4, -0.5e-4, Math.toRadians(0.), Math.toRadians(270.),
                                   Math.toRadians(5.300), CircularOrbit.MEAN_LONGITUDE_ARGUMENT, 
                                   FramesFactory.getEME2000(), date, mu);
        final LofOffset upsideDown = new LofOffset(RotationOrder.XYX, Math.PI, 0, 0);
        try {
            final LofOffsetPointing pointing = new LofOffsetPointing(earthSpheric, upsideDown, Vector3D.PLUS_K);
            pointing.getObservedGroundPoint(circ.getDate(), circ.getPVCoordinates(), circ.getFrame());
            fail("an exception should have been thrown");
        } catch (OrekitException oe) {
            // expected behavior
        } catch (Exception e) {
            fail("wrong exception caught");
        }
    }

    @Before
    public void setUp() {
        try {

            String root = getClass().getClassLoader().getResource("regular-data").getPath();
            System.setProperty(DataProvidersManager.OREKIT_DATA_PATH, root);

            // Computation date
            date = new AbsoluteDate(new DateComponents(2008, 04, 07),
                                    TimeComponents.H00,
                                    TimeScalesFactory.getUTC());

            // Body mu
            mu = 3.9860047e14;
            
            // Reference frame = ITRF 2005
            frameItrf2005 = FramesFactory.getITRF2005(true);

            // Elliptic earth shape
            earthSpheric =
                new OneAxisEllipsoid(6378136.460, 0., frameItrf2005);
            
        } catch (OrekitException oe) {
            fail(oe.getMessage());
        }

    }

    @After
    public void tearDown() {
        date = null;
        frameItrf2005 = null;
        earthSpheric = null;
    }
}

