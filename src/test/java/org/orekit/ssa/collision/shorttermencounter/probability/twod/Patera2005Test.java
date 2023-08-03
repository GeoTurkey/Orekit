/* Copyright 2002-2023 CS GROUP
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
package org.orekit.ssa.collision.shorttermencounter.probability.twod;

import java.io.IOException;
import java.util.List;

import org.hipparchus.Field;
import org.hipparchus.analysis.differentiation.DSFactory;
import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.analysis.integration.FieldTrapezoidIntegrator;
import org.hipparchus.analysis.integration.TrapezoidIntegrator;
import org.hipparchus.linear.FieldMatrix;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.Binary64;
import org.hipparchus.util.Binary64Field;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.orekit.Utils;
import org.orekit.data.DataSource;
import org.orekit.files.ccsds.ndm.ParserBuilder;
import org.orekit.files.ccsds.ndm.cdm.Cdm;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.LOFType;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.FieldCartesianOrbit;
import org.orekit.orbits.FieldOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.propagation.FieldStateCovariance;
import org.orekit.propagation.StateCovariance;
import org.orekit.ssa.collision.shorttermencounter.probability.twod.armellinutils.ArmellinDataLoader;
import org.orekit.ssa.collision.shorttermencounter.probability.twod.armellinutils.ArmellinDataRow;
import org.orekit.ssa.metrics.FieldProbabilityOfCollision;
import org.orekit.ssa.metrics.ProbabilityOfCollision;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.FieldAbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.FieldPVCoordinates;

class Patera2005Test {

    /** Patera method (2005) to compute probability of collision. */
    private final ShortTermEncounter2DPOCMethod method = new Patera2005();

    @BeforeAll
    static void initializeOrekitData() {
        Utils.setDataRoot("regular-data");
    }

    @Test
    @DisplayName("Chan test case 01")
    void ChanTestCase01() {
        // GIVEN
        final double xm     = 0;
        final double ym     = 10;
        final double sigmaX = 25;
        final double sigmaY = 50;
        final double radius = 5;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(9.741e-3, result.getValue(), 1e-6);
    }

    @Test
    @DisplayName("Chan test case 02")
    void ChanTestCase02() {
        // GIVEN
        final double xm     = 10;
        final double ym     = 0;
        final double sigmaX = 25;
        final double sigmaY = 50;
        final double radius = 5;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(9.181e-3, result.getValue(), 1e-6);
    }

    @Test
    @DisplayName("Chan test case 03")
    void ChanTestCase03() {
        // GIVEN
        final double xm     = 0;
        final double ym     = 10;
        final double sigmaX = 25;
        final double sigmaY = 75;
        final double radius = 5;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(6.571e-3, result.getValue(), 1e-6);
    }

    @Test
    @DisplayName("Chan test case 04")
    void ChanTestCase04() {
        // GIVEN
        final double xm     = 10;
        final double ym     = 0;
        final double sigmaX = 25;
        final double sigmaY = 75;
        final double radius = 5;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(6.125e-3, result.getValue(), 1e-6);
    }

    @Test
    @DisplayName("Chan test case 05")
    void ChanTestCase05() {
        // GIVEN
        final double xm     = 0;
        final double ym     = 1000;
        final double sigmaX = 1000;
        final double sigmaY = 3000;
        final double radius = 10;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(1.577e-5, result.getValue(), 1e-8);
    }

    @Test
    @DisplayName("Chan test case 06")
    void ChanTestCase06() {
        // GIVEN
        final double xm     = 1000;
        final double ym     = 0;
        final double sigmaX = 1000;
        final double sigmaY = 3000;
        final double radius = 10;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(1.011e-5, result.getValue(), 1e-8);
    }

    @Test
    @DisplayName("Chan test case 07")
    void ChanTestCase07() {
        // GIVEN
        final double xm     = 0;
        final double ym     = 10000;
        final double sigmaX = 1000;
        final double sigmaY = 3000;
        final double radius = 10;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(6.443e-8, result.getValue(), 1e-11);
    }

    @Test
    @DisplayName("Chan test case 08")
    void ChanTestCase08() {
        // GIVEN
        final double xm     = 10000;
        final double ym     = 0;
        final double sigmaX = 1000;
        final double sigmaY = 3000;
        final double radius = 10;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(3.219e-27, result.getValue(), 1e-30);
    }

    @Test
    @DisplayName("Chan test case 09")
    void ChanTestCase09() {
        // GIVEN
        final double xm     = 0;
        final double ym     = 10000;
        final double sigmaX = 1000;
        final double sigmaY = 10000;
        final double radius = 10;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(3.033e-6, result.getValue(), 1e-9);
    }

    @Test
    @DisplayName("Chan test case 10")
    void ChanTestCase10() {
        // GIVEN
        final double xm     = 10000;
        final double ym     = 0;
        final double sigmaX = 1000;
        final double sigmaY = 10000;
        final double radius = 10;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(9.656e-28, result.getValue(), 1e-31);
    }

    @Test
    @DisplayName("Chan test case 11")
    void ChanTestCase11() {
        // GIVEN
        final double xm     = 0;
        final double ym     = 5000;
        final double sigmaX = 1000;
        final double sigmaY = 3000;
        final double radius = 50;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(1.039e-4, result.getValue(), 1e-7);
    }

    @Test
    @DisplayName("Chan test case 12")
    void ChanTestCase12() {
        // GIVEN
        final double xm     = 5000;
        final double ym     = 0;
        final double sigmaX = 1000;
        final double sigmaY = 3000;
        final double radius = 50;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(1.564e-9, result.getValue(), 1e-12);
    }

    @Test
    @DisplayName("CSM test case 1")
    void CsmTestCase1() {
        // GIVEN
        final double xm     = 84.875546;
        final double ym     = 60.583685;
        final double sigmaX = 57.918666;
        final double sigmaY = 152.8814468;
        final double radius = 10.3;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(1.9002e-3, result.getValue(), 1e-7);
    }

    @Test
    @DisplayName("CSM test case 2")
    void CsmTestCase2() {
        // GIVEN
        final double xm     = -81.618369;
        final double ym     = 115.055899;
        final double sigmaX = 15.988242;
        final double sigmaY = 5756.840725;
        final double radius = 1.3;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(2.0553e-11, result.getValue(), 1e-15);
    }

    @Test
    @DisplayName("CSM test case 3")
    void CsmTestCase3() {
        // GIVEN
        final double xm     = 102.177247;
        final double ym     = 693.405893;
        final double sigmaX = 94.230921;
        final double sigmaY = 643.409272;
        final double radius = 5.3;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(7.2003e-5, result.getValue(), 1e-9);
    }

    @Test
    @DisplayName("CDM test case 1")
    void CdmTestCase1() {
        // GIVEN
        final double xm     = -752.672701;
        final double ym     = 644.939441;
        final double sigmaX = 445.859950;
        final double sigmaY = 6095.858688;
        final double radius = 3.5;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(5.3904e-7, result.getValue(), 1e-11);
    }

    @Test
    @DisplayName("CDM test case 2")
    void CdmTestCase2() {
        // GIVEN
        final double xm     = -692.362272;
        final double ym     = 4475.456261;
        final double sigmaX = 193.454603;
        final double sigmaY = 562.027293;
        final double radius = 13.2;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(2.2795e-20, result.getValue(), 1e-24);
    }

    @Test
    @DisplayName("Alfano test case 3")
    void AlfanoTestCase3() {
        // GIVEN
        final double xm     = -3.8872073;
        final double ym     = 0.1591646;
        final double sigmaX = 1.4101830;
        final double sigmaY = 114.2585190;
        final double radius = 15;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(1.0038e-1, result.getValue(), 1e-5);
    }

    @Disabled("Alfano test case 5 (Patera2005 method) : This test is numerically unstable and is ignored as it doesn't " +
            "use realistic values. More info about this case in Romain SERRA thesis : \"Romain Serra. Opérations de proximité en orbite :\n"
            + " * évaluation du risque de collision et calcul de manoeuvres optimales pour l’évitement et le rendez-vous. Automatique /\n"
            + " * Robotique. INSA de Toulouse, 2015. Français. NNT : 2015ISAT0035. tel-01261497\"")
    @Test
    @DisplayName("Alfano test case 5")
    void AlfanoTestCase5() {
        // GIVEN
        final double xm     = -1.2217895;
        final double ym     = 2.1230067;
        final double sigmaX = 0.0373279;
        final double sigmaY = 177.8109003;
        final double radius = 10;

        // WHEN
        final ProbabilityOfCollision result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(4.4509e-2, result.getValue(), 1e-6);
    }

    @Test
    @DisplayName(
            "Test Patera2005 special function on Roberto Armellin slightly modified case 210 in his data available here :" +
                    " github.com/arma1978/conjunction. The goal is to have a case where combined radius = miss distance and"
                    +
                    " case 210 was already very close." +
                    " We expect to find a probability very similar to what is GIVEN in his data")
    void testReturnValueCloseToUnmodifiedCase210() throws IOException {
        // GIVEN
        final int                           rowIndex = 209;
        final List<ArmellinDataRow>         rows     = ArmellinDataLoader.load();
        final ShortTermEncounter2DPOCMethod patera   = new Patera2005();

        final Frame        frame = FramesFactory.getGCRF();
        final AbsoluteDate date  = new AbsoluteDate();
        final double       mu    = Constants.IERS2010_EARTH_MU;

        // We force the combined radius to be equal to the miss distance (divide by two and change km to m)
        final double halfCombinedRadius = rows.get(rowIndex).getMissDistance() * 500;

        final Orbit primary = new CartesianOrbit(rows.get(rowIndex).getPrimaryPVCoordinates(),
                                                 frame, date, mu);
        final RealMatrix primaryCovarianceMatrix = rows.get(rowIndex).getPrimaryCovarianceMatrixInPrimaryRTN();
        final StateCovariance primaryCovariance =
                new StateCovariance(primaryCovarianceMatrix, date, LOFType.QSW_INERTIAL);

        final Orbit secondary = new CartesianOrbit(
                rows.get(rowIndex).getSecondaryPVCoordinates(), frame, date, mu);
        final RealMatrix secondaryCovarianceMatrix = rows.get(rowIndex)
                                                         .getSecondaryCovarianceMatrixInSecondaryRTN();
        final StateCovariance secondaryCovariance = new StateCovariance(secondaryCovarianceMatrix, date,
                                                                        LOFType.QSW_INERTIAL);

        // WHEN
        final ProbabilityOfCollision pateraResult =
                patera.compute(primary, primaryCovariance, halfCombinedRadius, secondary, secondaryCovariance,
                               halfCombinedRadius);

        // THEN
        Assertions.assertEquals(0.0012768565223002992, pateraResult.getValue(), 1e-19);

    }

    @Test
    @DisplayName(
            "Test probability from a real CDM. It has been mentioned in https://forum.orekit.org/t/collision-package-in-development/2638/16"
                    + " and define an expected probability of collision of 0.004450713 which is higher than the value found using"
                    + " the methods provided in this ssa package because the CSpOC use the projection of a square containing the"
                    + " collision disk instead of the collision disk directly. Hence an overestimation of the probability of"
                    + " collision. This test serves as a non regression test")
    void testComputeProbabilityFromACdm() {

        // GIVEN
        final String cdmPath   = "/collision-resources/ION_SCV8_vs_STARLINK_1233.txt";
        final DataSource  data = new DataSource(cdmPath, () -> getClass().getResourceAsStream(cdmPath));
        final Cdm         cdm  = new ParserBuilder().buildCdmParser().parseMessage(data);

        // Radii taken from comments in the conjunction data message
        final double primaryRadius   = 5;
        final double secondaryRadius = 5;

        final AbstractShortTermEncounter1DNumerical2DPOCMethod customMethod = new Patera2005();

        // WHEN
        final ProbabilityOfCollision result = customMethod.compute(cdm, primaryRadius, secondaryRadius,
                                                                   new TrapezoidIntegrator(), 50, 1e-15);

        // THEN
        Assertions.assertEquals(0.003496517644384083, result.getValue(), 1e-18);
    }

    @Test
    @DisplayName("Chan test case 01 Field version")
    void ChanTestCase01Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(0);
        final Binary64 ym     = new Binary64(10);
        final Binary64 sigmaX = new Binary64(25);
        final Binary64 sigmaY = new Binary64(50);
        final Binary64 radius = new Binary64(5);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(9.741e-3, result.getValue().getReal(), 1e-6);
    }

    @Test
    @DisplayName("Chan test case 02 Field version")
    void ChanTestCase02Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(10);
        final Binary64 ym     = new Binary64(0);
        final Binary64 sigmaX = new Binary64(25);
        final Binary64 sigmaY = new Binary64(50);
        final Binary64 radius = new Binary64(5);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(9.181e-3, result.getValue().getReal(), 1e-6);
    }

    @Test
    @DisplayName("Chan test case 03 Field version")
    void ChanTestCase03Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(0);
        final Binary64 ym     = new Binary64(10);
        final Binary64 sigmaX = new Binary64(25);
        final Binary64 sigmaY = new Binary64(75);
        final Binary64 radius = new Binary64(5);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(6.571e-3, result.getValue().getReal(), 1e-6);
    }

    @Test
    @DisplayName("Chan test case 04 Field version")
    void ChanTestCase04Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(10);
        final Binary64 ym     = new Binary64(0);
        final Binary64 sigmaX = new Binary64(25);
        final Binary64 sigmaY = new Binary64(75);
        final Binary64 radius = new Binary64(5);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(6.125e-3, result.getValue().getReal(), 1e-6);
    }

    @Test
    @DisplayName("Chan test case 05 Field version")
    void ChanTestCase05Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(0);
        final Binary64 ym     = new Binary64(1000);
        final Binary64 sigmaX = new Binary64(1000);
        final Binary64 sigmaY = new Binary64(3000);
        final Binary64 radius = new Binary64(10);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(1.577e-5, result.getValue().getReal(), 1e-8);
    }

    @Test
    @DisplayName("Chan test case 06 Field version")
    void ChanTestCase06Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(1000);
        final Binary64 ym     = new Binary64(0);
        final Binary64 sigmaX = new Binary64(1000);
        final Binary64 sigmaY = new Binary64(3000);
        final Binary64 radius = new Binary64(10);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(1.011e-5, result.getValue().getReal(), 1e-8);
    }

    @Test
    @DisplayName("Chan test case 07 Field version")
    void ChanTestCase07Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(0);
        final Binary64 ym     = new Binary64(10000);
        final Binary64 sigmaX = new Binary64(1000);
        final Binary64 sigmaY = new Binary64(3000);
        final Binary64 radius = new Binary64(10);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(6.443e-8, result.getValue().getReal(), 1e-11);
    }

    @Test
    @DisplayName("Chan test case 08 Field version")
    void ChanTestCase08Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(10000);
        final Binary64 ym     = new Binary64(0);
        final Binary64 sigmaX = new Binary64(1000);
        final Binary64 sigmaY = new Binary64(3000);
        final Binary64 radius = new Binary64(10);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(3.219e-27, result.getValue().getReal(), 1e-30);
    }

    @Test
    @DisplayName("Chan test case 09 Field version")
    void ChanTestCase09Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(0);
        final Binary64 ym     = new Binary64(10000);
        final Binary64 sigmaX = new Binary64(1000);
        final Binary64 sigmaY = new Binary64(10000);
        final Binary64 radius = new Binary64(10);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(3.033e-6, result.getValue().getReal(), 1e-9);
    }

    @Test
    @DisplayName("Chan test case 10 Field version")
    void ChanTestCase10Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(10000);
        final Binary64 ym     = new Binary64(0);
        final Binary64 sigmaX = new Binary64(1000);
        final Binary64 sigmaY = new Binary64(10000);
        final Binary64 radius = new Binary64(10);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(9.656e-28, result.getValue().getReal(), 1e-31);
    }

    @Test
    @DisplayName("Chan test case 11 Field version")
    void ChanTestCase11Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(0);
        final Binary64 ym     = new Binary64(5000);
        final Binary64 sigmaX = new Binary64(1000);
        final Binary64 sigmaY = new Binary64(3000);
        final Binary64 radius = new Binary64(50);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(1.039e-4, result.getValue().getReal(), 1e-7);
    }

    @Test
    @DisplayName("Chan test case 12 Field version")
    void ChanTestCase12Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(5000);
        final Binary64 ym     = new Binary64(0);
        final Binary64 sigmaX = new Binary64(1000);
        final Binary64 sigmaY = new Binary64(3000);
        final Binary64 radius = new Binary64(50);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(1.564e-9, result.getValue().getReal(), 1e-12);
    }

    @Test
    @DisplayName("CSM test case 1 Field version")
    void CsmTestCase1Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(84.875546);
        final Binary64 ym     = new Binary64(60.583685);
        final Binary64 sigmaX = new Binary64(57.918666);
        final Binary64 sigmaY = new Binary64(152.8814468);
        final Binary64 radius = new Binary64(10.3);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(1.9002e-3, result.getValue().getReal(), 1e-7);
    }

    @Test
    @DisplayName("CSM test case 2 Field version")
    void CsmTestCase2Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(-81.618369);
        final Binary64 ym     = new Binary64(115.055899);
        final Binary64 sigmaX = new Binary64(15.988242);
        final Binary64 sigmaY = new Binary64(5756.840725);
        final Binary64 radius = new Binary64(1.3);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(2.0553e-11, result.getValue().getReal(), 1e-15);
    }

    @Test
    @DisplayName("CSM test case 3 Field version")
    void CsmTestCase3Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(102.177247);
        final Binary64 ym     = new Binary64(693.405893);
        final Binary64 sigmaX = new Binary64(94.230921);
        final Binary64 sigmaY = new Binary64(643.409272);
        final Binary64 radius = new Binary64(5.3);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(7.2003e-5, result.getValue().getReal(), 1e-9);
    }

    @Test
    @DisplayName("CDM test case 1 Field version")
    void CdmTestCase1Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(-752.672701);
        final Binary64 ym     = new Binary64(644.939441);
        final Binary64 sigmaX = new Binary64(445.859950);
        final Binary64 sigmaY = new Binary64(6095.858688);
        final Binary64 radius = new Binary64(3.5);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(5.3904e-7, result.getValue().getReal(), 1e-11);
    }

    @Test
    @DisplayName("CDM test case 2 Field version")
    void CdmTestCase2Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(-692.362272);
        final Binary64 ym     = new Binary64(4475.456261);
        final Binary64 sigmaX = new Binary64(193.454603);
        final Binary64 sigmaY = new Binary64(562.027293);
        final Binary64 radius = new Binary64(13.2);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(2.2795e-20, result.getValue().getReal(), 1e-24);
    }

    @Test
    @DisplayName("Alfano test case 3 Field version")
    void AlfanoTestCase3Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(-3.8872073);
        final Binary64 ym     = new Binary64(0.1591646);
        final Binary64 sigmaX = new Binary64(1.4101830);
        final Binary64 sigmaY = new Binary64(114.2585190);
        final Binary64 radius = new Binary64(15);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(1.0038e-1, result.getValue().getReal(), 1e-5);
    }

    @Disabled(
            "Alfano test case 5 (field version) (Patera2005 method) : This test is numerically unstable and is ignored as it doesn't "
                    +
                    "use realistic values. More info about this case in Romain SERRA thesis : \"Romain Serra. Opérations de proximité en orbite :\n"
                    + " * évaluation du risque de collision et calcul de manoeuvres optimales pour l’évitement et le rendez-vous. Automatique /\n"
                    + " * Robotique. INSA de Toulouse, 2015. Français. NNT : 2015ISAT0035. tel-01261497\"")
    @Test
    @DisplayName("Alfano test case 5 Field version")
    void AlfanoTestCase5Field() {
        // GIVEN
        final Binary64 xm     = new Binary64(-1.2217895);
        final Binary64 ym     = new Binary64(2.1230067);
        final Binary64 sigmaX = new Binary64(0.0373279);
        final Binary64 sigmaY = new Binary64(177.8109003);
        final Binary64 radius = new Binary64(10);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = method.compute(xm, ym, sigmaX, sigmaY, radius);

        // THEN
        Assertions.assertEquals(4.4509e-2, result.getValue().getReal(), 1e-6);
    }

    @Test
    @DisplayName("Test impact on the probability of collision of a slight difference in combined radius in Chan test case 4")
    void testReturnExpectedValueWhenIntroducingSmallChangeOnCombinedRadius() {
        // GIVEN
        final DSFactory factory = new DSFactory(1, 10);

        final double xmNominal     = 10;
        final double ymNominal     = 0;
        final double sigmaXNominal = 25;
        final double sigmaYNominal = 75;
        final double radiusNominal = 5;
        final double dRadius       = 2.5;

        final DerivativeStructure xm     = factory.constant(xmNominal);
        final DerivativeStructure ym     = factory.constant(ymNominal);
        final DerivativeStructure sigmaX = factory.constant(sigmaXNominal);
        final DerivativeStructure sigmaY = factory.constant(sigmaYNominal);
        final DerivativeStructure radius = factory.variable(0, radiusNominal);

        // WHEN
        final FieldProbabilityOfCollision<DerivativeStructure> resultNominal =
                method.compute(xm, ym, sigmaX, sigmaY, radius);
        final double taylorResult = resultNominal.getValue().taylor(dRadius);
        final double exactResult =
                method.compute(xmNominal, ymNominal, sigmaXNominal, sigmaYNominal, radiusNominal + dRadius).getValue();

        // THEN
        Assertions.assertEquals(6.1e-3, resultNominal.getValue().getReal(), 1e-4);
        Assertions.assertEquals(exactResult, taylorResult, 1e-16);
    }

    @Test
    @DisplayName(
            "Test Patera2005 special function on Roberto Armellin slightly modified case 210 in his data available here :" +
                    " github.com/arma1978/conjunction. The goal is to have a case where combined radius = miss distance and"
                    +
                    " case 210 was already very close." +
                    " We expect to find a probability very similar to what is GIVEN in his data")
    void testReturnValueCloseToUnmodifiedCase210Field() throws IOException {
        // GIVEN
        final int                           rowIndex = 209;
        final List<ArmellinDataRow>         rows     = ArmellinDataLoader.load();
        final ShortTermEncounter2DPOCMethod patera   = new Patera2005();

        final Field<Binary64>             field = Binary64Field.getInstance();
        final Frame                       frame = FramesFactory.getGCRF();
        final FieldAbsoluteDate<Binary64> date  = new FieldAbsoluteDate<>(field);
        final Binary64                    mu    = new Binary64(Constants.IERS2010_EARTH_MU);

        // We force the combined radius to be equal to the miss distance (divide by two and change km to m)
        final Binary64 halfCombinedRadius = new Binary64(rows.get(rowIndex).getMissDistance() * 500);

        // Primary
        final FieldOrbit<Binary64> primary =
                new FieldCartesianOrbit<>(new FieldPVCoordinates<>(field, rows.get(rowIndex).getPrimaryPVCoordinates()),
                                          frame, date, mu);
        final FieldMatrix<Binary64> primaryCovarianceMatrix =
                rows.get(rowIndex).getPrimaryFieldCovarianceMatrixInPrimaryRTN();
        final FieldStateCovariance<Binary64> primaryCovariance =
                new FieldStateCovariance<>(primaryCovarianceMatrix, date, LOFType.QSW_INERTIAL);

        // Secondary
        final FieldOrbit<Binary64> secondary = new FieldCartesianOrbit<>(
                new FieldPVCoordinates<>(field, rows.get(rowIndex).getSecondaryPVCoordinates()), frame, date, mu);
        final FieldMatrix<Binary64> secondaryCovarianceMatrix =
                rows.get(rowIndex).getSecondaryFieldCovarianceMatrixInPrimaryRTN();
        final FieldStateCovariance<Binary64> secondaryCovariance =
                new FieldStateCovariance<>(secondaryCovarianceMatrix, date, LOFType.QSW_INERTIAL);

        // WHEN
        final FieldProbabilityOfCollision<Binary64> pateraResult =
                patera.compute(primary, primaryCovariance, halfCombinedRadius, secondary, secondaryCovariance,
                               halfCombinedRadius);

        // THEN
        Assertions.assertEquals(0.0012768565222964375, pateraResult.getValue().getReal(), 1e-19);

    }

    @Test
    @DisplayName("Test probability from a custom CDM. Initial frame is in ITRF so it has been " +
            "modified to GCRF. Hence we do not expect to find the same probability of collision as the one in the CDM." +
            "This is a non regression test." +
            "There is a negligible difference with the non field version because the combined covariance matrix is not " +
            "diagonalized using the same method")
    void testComputeProbabilityFromACdmField() {

        // GIVEN
        final String cdmPath   = "/collision-resources/ION_SCV8_vs_STARLINK_1233.txt";
        final DataSource  data = new DataSource(cdmPath, () -> getClass().getResourceAsStream(cdmPath));
        final Cdm         cdm  = new ParserBuilder().buildCdmParser().parseMessage(data);

        final Field<Binary64> field = Binary64Field.getInstance();

        // Radii taken from comments in the conjunction data message
        final Binary64 primaryRadius   = new Binary64(5);
        final Binary64 secondaryRadius = new Binary64(5);

        final AbstractShortTermEncounter1DNumerical2DPOCMethod customMethod = new Patera2005();

        // WHEN
        final FieldProbabilityOfCollision<Binary64> result = customMethod.compute(cdm, primaryRadius, secondaryRadius,
                                                                                  new FieldTrapezoidIntegrator<>(field),
                                                                                  55,
                                                                                  1e-15);

        // THEN
        Assertions.assertEquals(0.0034965176443840836, result.getValue().getReal(), 1e-19);
    }
}
