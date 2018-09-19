/* Copyright 2002-2018 CS Systèmes d'Information
 * Licensed to CS Systèmes d'Information (CS) under one or more
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
package org.orekit.gnss.attitude;

import org.junit.Test;
import org.orekit.frames.Frame;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.ExtendedPVCoordinatesProvider;


public class GPSBlockIIRTest extends AbstractGNSSAttitudeProviderTest {

    protected GNSSAttitudeProvider createProvider(final AbsoluteDate validityStart,
                                                  final AbsoluteDate validityEnd,
                                                  final ExtendedPVCoordinatesProvider sun,
                                                  final Frame inertialFrame,
                                                  final int prnNumber) {
        return new GPSBlockIIR(validityStart, validityEnd, sun, inertialFrame);
    }

    @Test
    public void testLargeNegativeBeta() {
        doTestAxes("beta-large-negative-BLOCK-IIR.txt", 1.5e-15, 1.2e-15, 8.8e-16);
    }

    @Test
    public void testSmallNegativeBeta() {
        doTestAxes("beta-small-negative-BLOCK-IIR.txt", 3.1e-13, 3.1e-13, 9.1e-16);
    }

    @Test
    public void testCrossingBeta() {
        doTestAxes("beta-crossing-BLOCK-IIR.txt", 5.2e-5, 5.2e-5, 4.9e-16);
    }

    @Test
    public void testSmallPositiveBeta() {
        doTestAxes("beta-small-positive-BLOCK-IIR.txt", 8.0e-13, 8.0e-13, 7.9e-16);
    }

    @Test
    public void testLargePositiveBeta() {
        doTestAxes("beta-large-positive-BLOCK-IIR.txt", 1.3e-15, 7.0e-15, 8.5e-16);
    }

}
