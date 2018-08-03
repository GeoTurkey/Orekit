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
package org.orekit.files.ccsds;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.Utils;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitMessages;
import org.orekit.utils.IERSConventions;


public class CCSDSFrameTest {

    @Test
    public void testLOFFramesNotRegularFrames() {
        for (final CCSDSFrame frame : CCSDSFrame.values()) {
            if (frame.isLof()) {
                try {
                    frame.getFrame(IERSConventions.IERS_2010, true);
                    Assert.fail("an exception should have been thrown");
                } catch (OrekitException oe) {
                    Assert.assertEquals(OrekitMessages.CCSDS_INVALID_FRAME, oe.getSpecifier());
                }
            } else {
                Assert.assertNull(frame.getLofType());
                Assert.assertNotNull(frame.getFrame(IERSConventions.IERS_2010, true));
            }
        }
    }

    @Before
    public void setUp() {
        Utils.setDataRoot("regular-data");
    }

}
