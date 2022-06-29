/* Copyright 2002-2022 CS GROUP
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
package org.orekit.gnss.metric.messages.rtcm;

import java.util.ArrayList;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.gnss.metric.messages.rtcm.correction.Rtcm1243;
import org.orekit.gnss.metric.messages.rtcm.correction.RtcmCombinedCorrectionData;
import org.orekit.gnss.metric.parser.ByteArrayEncodedMessages;
import org.orekit.gnss.metric.parser.EncodedMessage;
import org.orekit.gnss.metric.parser.RtcmMessagesParser;

public class Rtcm1243Test {

    private double eps = 1.0e-13;

    private EncodedMessage message;

    private ArrayList<Integer> messages;

    @Before
    public void setUp() {

        final String m = "010011011011" +                       // Message Number: 1243
                         "00001111110011000111" +               // Galileo Epoch Time 1s
                         "0101" +                               // SSR Update Interval
                         "0" +                                  // Multiple Message Indicator
                         "0" +                                  // Satellite Reference Datum
                         "0111" +                               // IOD SSR
                         "0000111101101111" +                   // SSR Provider ID
                         "0001" +                               // SSR Solution ID
                         "000001" +                             // No. of Satellites
                         "000001" +                             // Satellite ID
                         "0010000100" +                         // Galileo IOD
                         "0000101011111101111111" +             // Delta Radial
                         "01001010111111011111" +               // Delta Along-Track
                         "01001010111111011111" +               // Delta Cross-Track
                         "000010101111110111111" +              // Dot Delta Radial
                         "0100101011111101111" +                // Dot Delta Along-Track
                         "0100101011111101111" +                // Dot Delta Cross-Track
                         "0011101011111101111111" +             // Delta Clock C0
                         "001110101111110111111" +              // Delta Clock C1
                         "0011101011111101111111000110000000";  // Delta Clock C2

        message = new ByteArrayEncodedMessages(byteArrayFromBinary(m));
        message.start();

        messages = new ArrayList<>();
        messages.add(1243);

    }

    @Test
    public void testPerfectValue() {
        final Rtcm1243 rtcm1243 = (Rtcm1243) new RtcmMessagesParser(messages).parse(message, false);

        // Verify size
        Assert.assertEquals(1,                            rtcm1243.getData().size());

        // Verify header
        Assert.assertEquals(1243,                         rtcm1243.getTypeCode());
        Assert.assertEquals(64711.0,                      rtcm1243.getHeader().getEpochTime1s(), eps);
        Assert.assertEquals(30.0,                         rtcm1243.getHeader().getSsrUpdateInterval().getUpdateInterval(), eps);
        Assert.assertEquals(0,                            rtcm1243.getHeader().getMultipleMessageIndicator());
        Assert.assertEquals(7,                            rtcm1243.getHeader().getIodSsr());
        Assert.assertEquals(3951,                         rtcm1243.getHeader().getSsrProviderId());
        Assert.assertEquals(1,                            rtcm1243.getHeader().getSsrSolutionId());
        Assert.assertEquals(1,                            rtcm1243.getHeader().getNumberOfSatellites());

        // Verify data for satellite E01
        final RtcmCombinedCorrectionData e01 = rtcm1243.getDataMap().get("E01").get(0);
        Assert.assertEquals(1,                            e01.getSatelliteID());
        Assert.assertEquals(132,                          e01.getGnssIod());
        Assert.assertEquals(18.0095,                      e01.getOrbitCorrection().getDeltaOrbitRadial(),        eps);
        Assert.assertEquals(122.8668,                     e01.getOrbitCorrection().getDeltaOrbitAlongTrack(),    eps);
        Assert.assertEquals(122.8668,                     e01.getOrbitCorrection().getDeltaOrbitCrossTrack(),    eps);
        Assert.assertEquals(0.090047,                     e01.getOrbitCorrection().getDotOrbitDeltaRadial(),     eps);
        Assert.assertEquals(0.614332,                     e01.getOrbitCorrection().getDotOrbitDeltaAlongTrack(), eps);
        Assert.assertEquals(0.614332,                     e01.getOrbitCorrection().getDotOrbitDeltaCrossTrack(), eps);
        Assert.assertEquals(96.6527,                      e01.getClockCorrection().getDeltaClockC0(),            eps);
        Assert.assertEquals(0.483263,                     e01.getClockCorrection().getDeltaClockC1(),            eps);
        Assert.assertEquals(0.61857734,                   e01.getClockCorrection().getDeltaClockC2(),            eps);
    }

    private byte[] byteArrayFromBinary(String radix2Value) {
        final byte[] array = new byte[radix2Value.length() / 8];
        for (int i = 0; i < array.length; ++i) {
            for (int j = 0; j < 8; ++j) {
                if (radix2Value.charAt(8 * i + j) != '0') {
                    array[i] |= 0x1 << (7 - j);
                }
            }
        }
        return array;
    }

}
