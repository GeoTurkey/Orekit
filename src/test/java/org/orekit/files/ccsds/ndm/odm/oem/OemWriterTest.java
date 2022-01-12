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
package org.orekit.files.ccsds.ndm.odm.oem;

import org.junit.Test;
import org.orekit.files.ccsds.ndm.AbstractWriterTest;
import org.orekit.files.ccsds.ndm.ParsedUnitsBehavior;
import org.orekit.files.ccsds.ndm.ParserBuilder;
import org.orekit.files.ccsds.ndm.WriterBuilder;
import org.orekit.files.ccsds.section.Header;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;

public class OemWriterTest extends AbstractWriterTest<Header, OemSegment, Oem> {

    protected OemParser getParser() {
        return new ParserBuilder().
                   withMu(Constants.EIGEN5C_EARTH_MU).
                   withMissionReferenceDate(new AbsoluteDate("1996-12-17T00:00:00.000", TimeScalesFactory.getUTC())).
                   withParsedUnitsBehavior(ParsedUnitsBehavior.STRICT_COMPLIANCE).
                   buildOemParser();
    }

    protected OemWriter getWriter() {
        return new WriterBuilder().
                   withMissionReferenceDate(new AbsoluteDate("1996-12-17T00:00:00.000", TimeScalesFactory.getUTC())).
                   buildOemWriter();
    }

    @Test
    public void testWriteExample1() {
        doTest("/ccsds/odm/oem/OEMExample1.txt");
    }

    @Test
    public void testWriteExample2() {
        doTest("/ccsds/odm/oem/OEMExample2.txt");
    }

    @Test
    public void testWriteKvnExample3() {
        doTest("/ccsds/odm/oem/OEMExample3.txt");
    }

    @Test
    public void testWriteXmlExample3() {
        doTest("/ccsds/odm/oem/OEMExample3.txt");
    }

    @Test
    public void testWriteExample4() {
        doTest("/ccsds/odm/oem/OEMExample4.txt");
    }

    @Test
    public void testWriteExample5() {
        doTest("/ccsds/odm/oem/OEMExample5.txt");
    }

    @Test
    public void testWriteExample6() {
        doTest("/ccsds/odm/oem/OEMExample6.txt");
    }

    @Test
    public void testWriteExample8() {
        doTest("/ccsds/odm/oem/OEMExample8.txt");
    }

    @Test
    public void testIssue839() {
        doTest("/ccsds/odm/oem/OEM-Issue839.txt");
    }

}
