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
package org.orekit.propagation;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.BlockRealMatrix;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.orekit.Utils;
import org.orekit.errors.OrekitException;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.ICGEMFormatReader;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.LOFType;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;

public class StateCovarianceTest {

    private SpacecraftState initialState;
    private double[][]      initCov;
    final private double DEFAULT_VALLADO_THRESHOLD = 1e-6;

    /**
     * Unit test for the covariance frame transformation.
     */
    @Test
    public void testFrameConversion() {

        // Initialization
        setUp();

        // Define frames
        final Frame frameA = FramesFactory.getEME2000();
        final Frame frameB = FramesFactory.getTEME();

        // State covariance
        final StateCovariance reference = new StateCovariance(MatrixUtils.createRealMatrix(initCov), initialState.getDate(), frameA, OrbitType.CARTESIAN, PositionAngle.MEAN);

        // First transformation
        StateCovariance transformed = reference.changeCovarianceFrame(initialState.getOrbit(), frameB);

        // Second transformation
        transformed = transformed.changeCovarianceFrame(initialState.getOrbit(), frameA);

        // Verify
        compareCovariance(reference.getMatrix(), transformed.getMatrix(), 5.2e-15);

    }

    @Test
    public void testConstructor() {
        final AbsoluteDate initialDate          = new AbsoluteDate();
        final Frame        initialInertialFrame = FramesFactory.getGCRF();
    	final StateCovariance stateCovariance = new StateCovariance(getValladoInitialCovarianceMatrix(), initialDate, initialInertialFrame, OrbitType.CARTESIAN, PositionAngle.MEAN);
    	Assertions.assertEquals(OrbitType.CARTESIAN, stateCovariance.getOrbitType());
    	Assertions.assertEquals(PositionAngle.MEAN, stateCovariance.getPositionAngle());
    	Assertions.assertEquals(initialInertialFrame, stateCovariance.getFrame());
    	Assertions.assertNull(stateCovariance.getLOFType());
    	Assertions.assertEquals(initialDate, stateCovariance.getDate());
    }

    public void setUp() {
        Utils.setDataRoot("orbit-determination/february-2016:potential/icgem-format");
        GravityFieldFactory.addPotentialCoefficientsReader(new ICGEMFormatReader("eigen-6s-truncated", true));
        Orbit initialOrbit = new CartesianOrbit(
                new PVCoordinates(new Vector3D(7526993.581890527, -9646310.10026971, 1464110.4928112086),
                                  new Vector3D(3033.79456099698, 1715.265069098717, -4447.658745923895)),
                FramesFactory.getEME2000(),
                new AbsoluteDate("2016-02-13T16:00:00.000", TimeScalesFactory.getUTC()),
                Constants.WGS84_EARTH_MU);
        initialState = new SpacecraftState(initialOrbit);
        initCov = new double[][] {
                { 8.651816029e+01, 5.689987127e+01, -2.763870764e+01, -2.435617201e-02, 2.058274137e-02,
                        -5.872883051e-03 },
                { 5.689987127e+01, 7.070624321e+01, 1.367120909e+01, -6.112622013e-03, 7.623626008e-03,
                        -1.239413190e-02 },
                { -2.763870764e+01, 1.367120909e+01, 1.811858898e+02, 3.143798992e-02, -4.963106559e-02,
                        -7.420114385e-04 },
                { -2.435617201e-02, -6.112622013e-03, 3.143798992e-02, 4.657077389e-05, 1.469943634e-05,
                        3.328475593e-05 },
                { 2.058274137e-02, 7.623626008e-03, -4.963106559e-02, 1.469943634e-05, 3.950715934e-05,
                        2.516044258e-05 },
                { -5.872883051e-03, -1.239413190e-02, -7.420114385e-04, 3.328475593e-05, 2.516044258e-05,
                        3.547466120e-05 }
        };
    }

    /**
     * Compare two covariance matrices
     *
     * @param reference reference covariance
     * @param computed  computed covariance
     * @param threshold threshold for comparison
     */
    private void compareCovariance(final RealMatrix reference, final RealMatrix computed, final double threshold) {
        for (int row = 0; row < reference.getRowDimension(); row++) {
            for (int column = 0; column < reference.getColumnDimension(); column++) {
                if (reference.getEntry(row, column) == 0) {
                    Assertions.assertEquals(reference.getEntry(row, column), computed.getEntry(row, column),
                                            threshold);
                }
                else {
                    Assertions.assertEquals(reference.getEntry(row, column), computed.getEntry(row, column),
                                            FastMath.abs(threshold * reference.getEntry(row, column)));
                }
            }
        }
    }

    /**
     * In that particular case, the RTN local orbital frame is perfectly aligned with the inertial frame. Hence, the
     * frame conversion should return the same covariance matrix as input.
     */
    @Test
    @DisplayName("Test conversion from inertial frame to RTN local orbital frame")
    public void should_return_same_covariance_matrix() {

        // Given
        final AbsoluteDate initialDate          = new AbsoluteDate();
        final Frame        initialInertialFrame = FramesFactory.getGCRF();
        final double       mu                   = 398600e9;

        final PVCoordinates initialPV = new PVCoordinates(
                new Vector3D(6778000, 0, 0),
                new Vector3D(0, 7668.63, 0));

        final Orbit initialOrbit = new CartesianOrbit(initialPV, initialInertialFrame, initialDate, mu);

        final RealMatrix initialCovarianceInInertialFrame = new BlockRealMatrix(new double[][] {
                { 1, 0, 0, 0, 1e-5, 1e-4 },
                { 0, 1, 0, 0, 0, 1e-5 },
                { 0, 0, 1, 0, 0, 0 },
                { 0, 0, 0, 1e-3, 0, 0 },
                { 1e-5, 0, 0, 0, 1e-3, 0 },
                { 1e-4, 1e-5, 0, 0, 0, 1e-3 }
        });

        final StateCovariance stateCovariance = new StateCovariance(initialCovarianceInInertialFrame, initialDate, initialInertialFrame, OrbitType.CARTESIAN, PositionAngle.MEAN);
        // When
        final RealMatrix covarianceMatrixInRTN = stateCovariance.changeCovarianceFrame(initialOrbit, LOFType.QSW).getMatrix();

        // Then
        final RealMatrix expectedMatrixInRTN = new BlockRealMatrix(new double[][] {
                { 1, 0, 0, 0, 1e-5, 1e-4 },
                { 0, 1, 0, 0, 0, 1e-5 },
                { 0, 0, 1, 0, 0, 0 },
                { 0, 0, 0, 1e-3, 0, 0 },
                { 1e-5, 0, 0, 0, 1e-3, 0 },
                { 1e-4, 1e-5, 0, 0, 0, 1e-3 }
        });

        compareCovariance(covarianceMatrixInRTN, expectedMatrixInRTN, 1e-20);

    }

    /**
     * Unit test for the covariance type transformation.
     */
    @Test
    public void testTypeConversion() {

        // Initialization
        setUp();

        // Define orbit types
        final OrbitType cart = OrbitType.CARTESIAN;
        final OrbitType kep  = OrbitType.KEPLERIAN;

        // State covariance
        final StateCovariance reference = new StateCovariance(MatrixUtils.createRealMatrix(initCov), initialState.getDate(), initialState.getFrame(), cart, PositionAngle.MEAN);

        // First transformation
        StateCovariance transformed = reference.changeCovarianceType(initialState.getOrbit(), kep, PositionAngle.MEAN);

        // Second transformation
        transformed = transformed.changeCovarianceType(initialState.getOrbit(), cart, PositionAngle.MEAN);

        // Verify
        compareCovariance(reference.getMatrix(), transformed.getMatrix(), 3.5e-12);

    }

    @Test
    @DisplayName("Test covariance conversion from inertial frame to RTN local orbital frame")
    public void should_rotate_covariance_matrix_by_ninety_degrees() {

        // Given
        final AbsoluteDate initialDate          = new AbsoluteDate();
        final Frame        initialInertialFrame = FramesFactory.getGCRF();
        final double       mu                   = 398600e9;

        final PVCoordinates initialPV = new PVCoordinates(
                new Vector3D(0, 6778000, 0),
                new Vector3D(-7668.63, 0, 0));

        final Orbit initialOrbit = new CartesianOrbit(initialPV, initialInertialFrame, initialDate, mu);

        final RealMatrix initialCovarianceInInertialFrame = new BlockRealMatrix(new double[][] {
                { 1, 0, 0, 0, 0, 1e-5 },
                { 0, 1, 0, 0, 0, 0 },
                { 0, 0, 1, 0, 0, 0 },
                { 0, 0, 0, 1e-3, 0, 0 },
                { 0, 0, 0, 0, 1e-3, 0 },
                { 1e-5, 0, 0, 0, 0, 1e-3 }
        });

        final StateCovariance stateCovariance = new StateCovariance(initialCovarianceInInertialFrame, initialDate, initialInertialFrame, OrbitType.CARTESIAN, PositionAngle.MEAN);

        // When
        final RealMatrix convertedCovarianceMatrixInRTN = stateCovariance.changeCovarianceFrame(initialOrbit, LOFType.QSW).getMatrix();

        // Then
        // Expected covariance matrix obtained by rotation initial covariance matrix by 90 degrees
        final RealMatrix expectedMatrixInRTN = new BlockRealMatrix(new double[][] {
                { 1, 0, 0, 0, 0, 0 },
                { 0, 1, 0, 0, 0, -1e-5 },
                { 0, 0, 1, 0, 0, 0 },
                { 0, 0, 0, 1e-3, 0, 0 },
                { 0, 0, 0, 0, 1e-3, 0 },
                { 0, -1e-5, 0, 0, 0, 1e-3 }
        });

        compareCovariance(expectedMatrixInRTN, convertedCovarianceMatrixInRTN, 1e-20);
    }

    @Test
    @DisplayName("Test covariance conversion from RTN local orbital frame to inertial frame")
    public void should_rotate_covariance_matrix_by_minus_ninety_degrees() {

        // Given
        final AbsoluteDate initialDate          = new AbsoluteDate();
        final Frame        initialInertialFrame = FramesFactory.getGCRF();
        final double       mu                   = 398600e9;

        final PVCoordinates initialPV = new PVCoordinates(
                new Vector3D(0, 6778000, 0),
                new Vector3D(-7668.63, 0, 0));

        final Orbit initialOrbit = new CartesianOrbit(initialPV, initialInertialFrame, initialDate, mu);

        final RealMatrix initialCovarianceInRTN = new BlockRealMatrix(new double[][] {
                { 1, 0, 0, 0, 0, 0 },
                { 0, 1, 0, 0, 0, -1e-5 },
                { 0, 0, 1, 0, 0, 0 },
                { 0, 0, 0, 1e-3, 0, 0 },
                { 0, 0, 0, 0, 1e-3, 0 },
                { 0, -1e-5, 0, 0, 0, 1e-3 }
        });

        final StateCovariance stateCovariance = new StateCovariance(initialCovarianceInRTN, initialDate, LOFType.QSW);

        // When
        final RealMatrix convertedCovarianceMatrixInInertialFrame = stateCovariance.changeCovarianceFrame(initialOrbit, initialInertialFrame).getMatrix();

        // Then

        // Expected covariance matrix obtained by rotation initial covariance matrix by -90 degrees
        final RealMatrix expectedMatrixInInertialFrame = new BlockRealMatrix(new double[][] {
                { 1, 0, 0, 0, 0, 1e-5 },
                { 0, 1, 0, 0, 0, 0 },
                { 0, 0, 1, 0, 0, 0 },
                { 0, 0, 0, 1e-3, 0, 0 },
                { 0, 0, 0, 0, 1e-3, 0 },
                { 1e-5, 0, 0, 0, 0, 1e-3 }
        });

        compareCovariance(expectedMatrixInInertialFrame, convertedCovarianceMatrixInInertialFrame, 1e-20);

    }

    /**
     * This test is based on the following paper : Covariance Transformations for Satellite Flight Dynamics Operations
     * from David A. Vallado.
     * <p>
     * More specifically, we're using the initial covariance matrix from p.14 and compare the computed result with the
     * cartesian covariance in RSW from p.19.
     * <p>
     * Note that the followings local orbital frame are equivalent RSW=RTN=QSW.
     */
    @Test
    @DisplayName("Test covariance conversion Vallado test case : ECI cartesian to RTN")
    public void should_return_Vallado_RSW_covariance_matrix_from_ECI() {

        // Initialize Orekit
        Utils.setDataRoot("regular-data");

        // Given
        final AbsoluteDate  initialDate          = getValladoInitialDate();
        final PVCoordinates initialPV            = getValladoInitialPV();
        final Frame         initialInertialFrame = FramesFactory.getGCRF();
        final Orbit initialOrbit =
                new CartesianOrbit(initialPV, initialInertialFrame, initialDate, getValladoMu());

        final RealMatrix initialCovarianceMatrix = getValladoInitialCovarianceMatrix();

        final StateCovariance stateCovariance = new StateCovariance(initialCovarianceMatrix, initialDate, initialInertialFrame, OrbitType.CARTESIAN, PositionAngle.MEAN);
        // When
        final RealMatrix convertedCovarianceMatrixInRTN = stateCovariance.changeCovarianceFrame(initialOrbit, LOFType.QSW).getMatrix();

        // Then
        final RealMatrix expectedCovarianceMatrixInRTN = new BlockRealMatrix(new double[][] {
                { 9.918921e-001, 6.700644e-003, -2.878187e-003, 1.892086e-005, 6.700644e-005, -2.878187e-005 },
                { 6.700644e-003, 1.013730e+000, -1.019283e-002, 6.700644e-005, 2.372970e-004, -1.019283e-004 },
                { -2.878187e-003, -1.019283e-002, 9.943782e-001, -2.878187e-005, -1.019283e-004, 4.378217e-005 },
                { 1.892086e-005, 6.700644e-005, -2.878187e-005, 1.892086e-007, 6.700644e-007, -2.878187e-007 },
                { 6.700644e-005, 2.372970e-004, -1.019283e-004, 6.700644e-007, 2.372970e-006, -1.019283e-006 },
                { -2.878187e-005, -1.019283e-004, 4.378217e-005, -2.878187e-007, -1.019283e-006, 4.378217e-007 }
        });

        compareCovariance(expectedCovarianceMatrixInRTN, convertedCovarianceMatrixInRTN, DEFAULT_VALLADO_THRESHOLD);

    }

    private AbsoluteDate getValladoInitialDate() {
        return new AbsoluteDate(2000, 12, 15, 16, 58, 50.208, TimeScalesFactory.getUTC());
    }

    private PVCoordinates getValladoInitialPV() {
        return new PVCoordinates(
                new Vector3D(-605792.21660, -5870229.51108, 3493053.19896),
                new Vector3D(-1568.25429, -3702.34891, -6479.48395));
    }

    private double getValladoMu() {
        return Constants.IERS2010_EARTH_MU;
    }

    private RealMatrix getValladoInitialCovarianceMatrix() {
        return new BlockRealMatrix(new double[][] {
                { 1, 1e-2, 1e-2, 1e-4, 1e-4, 1e-4 },
                { 1e-2, 1, 1e-2, 1e-4, 1e-4, 1e-4 },
                { 1e-2, 1e-2, 1, 1e-4, 1e-4, 1e-4 },
                { 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6 },
                { 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6 },
                { 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6 }
        });
    }

    /**
     * This test is based on the following paper : Covariance Transformations for Satellite Flight Dynamics Operations
     * from David A. Vallado.
     * <p>
     * More specifically, we're using the initial covariance matrix from p.14 and compare the computed result with the
     * cartesian covariance in NTW from p.19.
     */
    @Test
    @DisplayName("Test covariance conversion Vallado test case : ECI cartesian to NTW")
    public void should_return_Vallado_NTW_covariance_matrix_from_ECI() {

        // Initialize orekit
        Utils.setDataRoot("regular-data");

        // Given
        final AbsoluteDate  initialDate          = getValladoInitialDate();
        final PVCoordinates initialPV            = getValladoInitialPV();
        final Frame         initialInertialFrame = FramesFactory.getGCRF();
        final Orbit initialOrbit =
                new CartesianOrbit(initialPV, initialInertialFrame, initialDate, getValladoMu());

        final RealMatrix initialCovarianceMatrix = getValladoInitialCovarianceMatrix();

        final StateCovariance stateCovariance = new StateCovariance(initialCovarianceMatrix, initialDate, initialInertialFrame, OrbitType.CARTESIAN, PositionAngle.MEAN);
        // When
        final RealMatrix convertedCovarianceMatrixInNTW =
        		stateCovariance.changeCovarianceFrame(initialOrbit, LOFType.NTW).getMatrix();

        // Then
        final RealMatrix expectedCovarianceMatrixInNTW = new BlockRealMatrix(new double[][] {
                { 9.918792e-001, 6.679546e-003, -2.868345e-003, 1.879167e-005, 6.679546e-005, -2.868345e-005 },
                { 6.679546e-003, 1.013743e+000, -1.019560e-002, 6.679546e-005, 2.374262e-004, -1.019560e-004 },
                { -2.868345e-003, -1.019560e-002, 9.943782e-001, -2.868345e-005, -1.019560e-004, 4.378217e-005 },
                { 1.879167e-005, 6.679546e-005, -2.868345e-005, 1.879167e-007, 6.679546e-007, -2.868345e-007 },
                { 6.679546e-005, 2.374262e-004, -1.019560e-004, 6.679546e-007, 2.374262e-006, -1.019560e-006 },
                { -2.868345e-005, -1.019560e-004, 4.378217e-005, -2.868345e-007, -1.019560e-006, 4.378217e-007 }
        });

        compareCovariance(expectedCovarianceMatrixInNTW, convertedCovarianceMatrixInNTW, DEFAULT_VALLADO_THRESHOLD);

    }

    /**
     * This test is based on the following paper : Covariance Transformations for Satellite Flight Dynamics Operations
     * from David A. Vallado.
     * <p>
     * More specifically, we're using the initial covariance matrix from p.14 as a reference to test multiple
     * conversions.
     * <p>
     * This test aims to verify the numerical precision after various conversions and serves as a non regression test
     * for future updates.
     * <p>
     * Also, note that the conversion from the RTN to TEME tests the fact that the orbit is initially expressed in GCRF
     * while we want the covariance expressed in TEME. Hence, it tests that the rotation from RTN to TEME needs to be
     * obtained by expressing the orbit PVCoordinates in the TEME frame (hence the use of orbit.gtPVCoordinates(frameOut)
     * ,see relevant changeCovarianceFrame method).
     */
    @Test
    @DisplayName("Test custom covariance conversion Vallado test case : GCRF -> TEME -> IRTF -> NTW -> RTN -> ITRF -> GCRF")
    public  void should_return_initial_covariance_after_multiple_conversion() {

        // Initialize orekit
        Utils.setDataRoot("regular-data");

        // Given
        final AbsoluteDate  initialDate          = getValladoInitialDate();
        final PVCoordinates initialPV            = getValladoInitialPV();
        final Frame         initialInertialFrame = FramesFactory.getGCRF();
        final Orbit initialOrbit =
                new CartesianOrbit(initialPV, initialInertialFrame, initialDate, getValladoMu());

        final RealMatrix initialCovarianceMatrixInGCRF = getValladoInitialCovarianceMatrix();

        final Frame teme = FramesFactory.getTEME();

        final Frame itrf = FramesFactory.getITRF(IERSConventions.IERS_2010, false);

        final StateCovariance stateCovariance = new StateCovariance(initialCovarianceMatrixInGCRF, initialDate, initialInertialFrame, OrbitType.CARTESIAN, PositionAngle.MEAN);

        // When
        // GCRF -> TEME
        final StateCovariance convertedCovarianceMatrixInTEME =
        		stateCovariance.changeCovarianceFrame(initialOrbit, teme);

        // TEME -> ITRF
        final StateCovariance convertedCovarianceMatrixInITRF =
        		convertedCovarianceMatrixInTEME.changeCovarianceFrame(initialOrbit, itrf);

        // ITRF -> NTW
        final StateCovariance convertedCovarianceMatrixInNTW =
        		convertedCovarianceMatrixInITRF.changeCovarianceFrame(initialOrbit, LOFType.NTW);

        // NTW -> RTN
        final StateCovariance convertedCovarianceMatrixInRTN =
        		convertedCovarianceMatrixInNTW.changeCovarianceFrame(initialOrbit, LOFType.QSW);

        // RTN -> ITRF
        final StateCovariance convertedCovarianceMatrixBackInITRF =
        		convertedCovarianceMatrixInRTN.changeCovarianceFrame(initialOrbit, itrf);

        // ITRF -> TEME
        final StateCovariance convertedCovarianceMatrixBackInTEME =
        		convertedCovarianceMatrixBackInITRF.changeCovarianceFrame(initialOrbit, teme);

        // TEME -> GCRF
        final StateCovariance convertedCovarianceMatrixInGCRF =
        		convertedCovarianceMatrixBackInTEME.changeCovarianceFrame(initialOrbit, initialInertialFrame);

        // Then
        final RealMatrix expectedCovarianceMatrixInGCRF = getValladoInitialCovarianceMatrix();

        compareCovariance(expectedCovarianceMatrixInGCRF, convertedCovarianceMatrixInGCRF.getMatrix(), 1e-12);

    }

    /**
     * This test is based on the following paper : Covariance Transformations for Satellite Flight Dynamics Operations
     * from David A. Vallado.
     * <p>
     * More specifically, we're using the initial covariance matrix from p.14 and compare the computed result with the
     * cartesian covariance in ECEF from p.18.
     * </p>
     * <p>
     * <b>BEWARE: It has been found that the earth rotation in this Vallado's case is given 1 million times slower than
     * the expected value. This has been corrected and the expected covariance matrix is now the covariance matrix
     * computed by Orekit given the similarities with Vallado's results. In addition, the small differences potentially
     * come from the custom EOP that Vallado uses. Hence, this test can be considered as a <u>non regression
     * test</u>.</b>
     * </p>
     */
    @Test
    @DisplayName("Test covariance conversion Vallado test case : ECI cartesian to PEF")
    public void should_return_Vallado_PEF_covariance_matrix_from_ECI() {

        // Initialize orekit
        Utils.setDataRoot("regular-data");

        // Given
        // Initialize orbit
        final AbsoluteDate  initialDate          = getValladoInitialDate();
        final PVCoordinates initialPV            = getValladoInitialPV();
        final Frame         initialInertialFrame = FramesFactory.getGCRF();

        final Orbit initialOrbit = new CartesianOrbit(initialPV, initialInertialFrame, initialDate, getValladoMu());

        // Initialize input covariance matrix
        final RealMatrix initialCovarianceMatrix = getValladoInitialCovarianceMatrix();

        // Initialize output frame
        final Frame outputFrame = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        final StateCovariance stateCovariance = new StateCovariance(initialCovarianceMatrix, initialDate, initialInertialFrame, OrbitType.CARTESIAN, PositionAngle.MEAN);
        // When
        final RealMatrix convertedCovarianceMatrixInITRF =
        		stateCovariance.changeCovarianceFrame(initialOrbit, outputFrame).getMatrix();

        // Then
        final RealMatrix expectedCovarianceMatrixInITRF = new BlockRealMatrix(new double[][] {
                { 9.9340005761276870e-01, 7.5124999798868530e-03, 5.8312675007359050e-03, 3.4548396261054936e-05,
                        2.6851237046859200e-06, 5.8312677693153940e-05 },
                { 7.5124999798868025e-03, 1.0065990293034541e+00, 1.2884310200351924e-02, 1.4852736004690684e-04,
                        1.6544247282904867e-04, 1.2884310644320954e-04 },
                { 5.8312675007359040e-03, 1.2884310200351924e-02, 1.0000009130837746e+00, 5.9252211072590390e-05,
                        1.2841787487219444e-04, 1.0000913090989617e-04 },
                { 3.4548396261054936e-05, 1.4852736004690686e-04, 5.9252211072590403e-05, 3.5631474857130520e-07,
                        7.6083489184819870e-07, 5.9252213790760030e-07 },
                { 2.6851237046859150e-06, 1.6544247282904864e-04, 1.2841787487219447e-04, 7.6083489184819880e-07,
                        1.6542289254142709e-06, 1.2841787929229964e-06 },
                { 5.8312677693153934e-05, 1.2884310644320950e-04, 1.0000913090989616e-04, 5.9252213790760020e-07,
                        1.2841787929229960e-06, 1.0000913098203875e-06 }
        });

        compareCovariance(expectedCovarianceMatrixInITRF, convertedCovarianceMatrixInITRF, 1e-20);

    }

    /**
     * This test is based on the following paper : Covariance Transformations for Satellite Flight Dynamics Operations
     * from David A. Vallado.
     * <p>
     * More specifically, we're using the initial covariance matrix from p.14 and compare the computed result with the
     * cartesian covariance in PEF from p.18.
     */
    @Test
    @DisplayName("Test covariance conversion Vallado test case : PEF cartesian to ECI")
    public void should_return_Vallado_ECI_covariance_matrix_from_PEF() {

        // Initialize orekit
        Utils.setDataRoot("regular-data");

        // Given
        final AbsoluteDate  initialDate          = getValladoInitialDate();
        final PVCoordinates initialPV            = getValladoInitialPV();
        final Frame         initialInertialFrame = FramesFactory.getGCRF();
        final Orbit initialOrbit =
                new CartesianOrbit(initialPV, initialInertialFrame, initialDate, getValladoMu());

        final RealMatrix initialCovarianceMatrixInPEF = new BlockRealMatrix(new double[][] {
                { 9.9340005761276870e-01, 7.5124999798868530e-03, 5.8312675007359050e-03, 3.4548396261054936e-05,
                        2.6851237046859200e-06, 5.8312677693153940e-05 },
                { 7.5124999798868025e-03, 1.0065990293034541e+00, 1.2884310200351924e-02, 1.4852736004690684e-04,
                        1.6544247282904867e-04, 1.2884310644320954e-04 },
                { 5.8312675007359040e-03, 1.2884310200351924e-02, 1.0000009130837746e+00, 5.9252211072590390e-05,
                        1.2841787487219444e-04, 1.0000913090989617e-04 },
                { 3.4548396261054936e-05, 1.4852736004690686e-04, 5.9252211072590403e-05, 3.5631474857130520e-07,
                        7.6083489184819870e-07, 5.9252213790760030e-07 },
                { 2.6851237046859150e-06, 1.6544247282904864e-04, 1.2841787487219447e-04, 7.6083489184819880e-07,
                        1.6542289254142709e-06, 1.2841787929229964e-06 },
                { 5.8312677693153934e-05, 1.2884310644320950e-04, 1.0000913090989616e-04, 5.9252213790760020e-07,
                        1.2841787929229960e-06, 1.0000913098203875e-06 }
        });

        final Frame inputFrame = FramesFactory.getITRF(IERSConventions.IERS_2010, false);

        // State covariance
        final StateCovariance stateCovariance = new StateCovariance(initialCovarianceMatrixInPEF, initialDate, inputFrame, OrbitType.CARTESIAN, PositionAngle.MEAN);

        // When
        final RealMatrix convertedCovarianceMatrixInECI = stateCovariance.changeCovarianceFrame(initialOrbit, initialInertialFrame).getMatrix();

        // Then
        final RealMatrix expectedCovarianceMatrixInECI = getValladoInitialCovarianceMatrix();

        compareCovariance(expectedCovarianceMatrixInECI, convertedCovarianceMatrixInECI, 1e-7);

    }

    /**
     * This test is based on the following paper : Covariance Transformations for Satellite Flight Dynamics Operations
     * from David A. Vallado.
     * <p>
     * More specifically, we're using the initial covariance matrix from p.14 and compare the computed result with the
     * cartesian covariance in MOD from p.17.
     * </p>
     */
    @Test
    @DisplayName("Test covariance conversion Vallado test case : ECI cartesian to MOD")
    public void should_return_Vallado_MOD_covariance_matrix_from_ECI() {

        // Initialize orekit
        Utils.setDataRoot("regular-data");

        // Given
        final AbsoluteDate  initialDate          = getValladoInitialDate();
        final PVCoordinates initialPV            = getValladoInitialPV();
        final Frame         initialInertialFrame = FramesFactory.getGCRF();
        final Orbit initialOrbit =
                new CartesianOrbit(initialPV, initialInertialFrame, initialDate, getValladoMu());

        final RealMatrix initialCovarianceMatrix = getValladoInitialCovarianceMatrix();

        final Frame outputFrame = FramesFactory.getMOD(IERSConventions.IERS_2010);

        final StateCovariance stateCovariance = new StateCovariance(initialCovarianceMatrix, initialDate, initialInertialFrame, OrbitType.CARTESIAN, PositionAngle.MEAN);
        // When
        final RealMatrix convertedCovarianceMatrixInMOD =
        		stateCovariance.changeCovarianceFrame(initialOrbit, outputFrame).getMatrix();

        // Then
        final RealMatrix expectedCovarianceMatrixInMOD = new BlockRealMatrix(new double[][] {
                { 9.999939e-001, 9.999070e-003, 9.997861e-003, 9.993866e-005, 9.999070e-005, 9.997861e-005 },
                { 9.999070e-003, 1.000004e+000, 1.000307e-002, 9.999070e-005, 1.000428e-004, 1.000307e-004 },
                { 9.997861e-003, 1.000307e-002, 1.000002e+000, 9.997861e-005, 1.000307e-004, 1.000186e-004 },
                { 9.993866e-005, 9.999070e-005, 9.997861e-005, 9.993866e-007, 9.999070e-007, 9.997861e-007 },
                { 9.999070e-005, 1.000428e-004, 1.000307e-004, 9.999070e-007, 1.000428e-006, 1.000307e-006 },
                { 9.997861e-005, 1.000307e-004, 1.000186e-004, 9.997861e-007, 1.000307e-006, 1.000186e-006 },
        });

        compareCovariance(expectedCovarianceMatrixInMOD, convertedCovarianceMatrixInMOD, DEFAULT_VALLADO_THRESHOLD);

    }

    /**
     * This test is based on the following paper : Covariance Transformations for Satellite Flight Dynamics Operations
     * from David A. Vallado.
     * <p>
     * More specifically, we're using the initial NTW covariance matrix from p.19 and compare the computed result with
     * the cartesian covariance in RSW from the same page.
     * </p>
     */
    @Test
    @DisplayName("Test covariance conversion from Vallado test case NTW to RSW")
    public void should_convert_Vallado_NTW_to_RSW() {

        // Initialize orekit
        Utils.setDataRoot("regular-data");

        // Given
        final AbsoluteDate  initialDate          = getValladoInitialDate();
        final PVCoordinates initialPV            = getValladoInitialPV();
        final Frame         initialInertialFrame = FramesFactory.getGCRF();
        final Orbit initialOrbit =
                new CartesianOrbit(initialPV, initialInertialFrame, initialDate, getValladoMu());

        final RealMatrix initialCovarianceMatrixInNTW = new BlockRealMatrix(new double[][] {
                { 9.918792e-001, 6.679546e-003, -2.868345e-003, 1.879167e-005, 6.679546e-005, -2.868345e-005 },
                { 6.679546e-003, 1.013743e+000, -1.019560e-002, 6.679546e-005, 2.374262e-004, -1.019560e-004 },
                { -2.868345e-003, -1.019560e-002, 9.943782e-001, -2.868345e-005, -1.019560e-004, 4.378217e-005 },
                { 1.879167e-005, 6.679546e-005, -2.868345e-005, 1.879167e-007, 6.679546e-007, -2.868345e-007 },
                { 6.679546e-005, 2.374262e-004, -1.019560e-004, 6.679546e-007, 2.374262e-006, -1.019560e-006 },
                { -2.868345e-005, -1.019560e-004, 4.378217e-005, -2.868345e-007, -1.019560e-006, 4.378217e-007 }
        });

        final StateCovariance stateCovariance = new StateCovariance(initialCovarianceMatrixInNTW, initialDate, LOFType.NTW);

        // When
        final RealMatrix convertedCovarianceMatrixInRTN =
        		stateCovariance.changeCovarianceFrame(initialOrbit, LOFType.QSW).getMatrix();

        // Then
        final RealMatrix expectedCovarianceMatrixInRTN = new BlockRealMatrix(new double[][] {
                { 9.918921e-001, 6.700644e-003, -2.878187e-003, 1.892086e-005, 6.700644e-005, -2.878187e-005 },
                { 6.700644e-003, 1.013730e+000, -1.019283e-002, 6.700644e-005, 2.372970e-004, -1.019283e-004 },
                { -2.878187e-003, -1.019283e-002, 9.943782e-001, -2.878187e-005, -1.019283e-004, 4.378217e-005 },
                { 1.892086e-005, 6.700644e-005, -2.878187e-005, 1.892086e-007, 6.700644e-007, -2.878187e-007 },
                { 6.700644e-005, 2.372970e-004, -1.019283e-004, 6.700644e-007, 2.372970e-006, -1.019283e-006 },
                { -2.878187e-005, -1.019283e-004, 4.378217e-005, -2.878187e-007, -1.019283e-006, 4.378217e-007 }
        });

        compareCovariance(expectedCovarianceMatrixInRTN, convertedCovarianceMatrixInRTN, DEFAULT_VALLADO_THRESHOLD);

    }

    @Test
    @DisplayName("Test shiftedBy method of StateCovariance")
    void Should_return_expected_shifted_state_covariance() {
        // Given


        // When

        // Then

    }

    /**
     * This test is based on the following paper : Covariance Transformations for Satellite Flight Dynamics Operations
     * from David A. Vallado.
     * <p>
     * More specifically, we're using the initial NTW covariance matrix from p.19 and compare the computed result with
     * the cartesian covariance in RSW from the same page.
     * </p>
     */
    @Test
    @DisplayName("Test thrown error if input frame is not pseudo-inertial and the covariance matrix is not expressed in cartesian elements")
    public void should_return_orekit_exception() {

        // Initialize orekit
        Utils.setDataRoot("regular-data");

        // Given
        final AbsoluteDate  initialDate          = getValladoInitialDate();
        final PVCoordinates initialPV            = getValladoInitialPV();
        final Frame         initialInertialFrame = FramesFactory.getGCRF();
        final Orbit initialOrbit =
                new CartesianOrbit(initialPV, initialInertialFrame, initialDate, getValladoMu());

        final RealMatrix randomCovarianceMatrix = new BlockRealMatrix(new double[][] {
                { 9.918792e-001, 6.679546e-003, -2.868345e-003, 1.879167e-005, 6.679546e-005, -2.868345e-005 },
                { 6.679546e-003, 1.013743e+000, -1.019560e-002, 6.679546e-005, 2.374262e-004, -1.019560e-004 },
                { -2.868345e-003, -1.019560e-002, 9.943782e-001, -2.868345e-005, -1.019560e-004, 4.378217e-005 },
                { 1.879167e-005, 6.679546e-005, -2.868345e-005, 1.879167e-007, 6.679546e-007, -2.868345e-007 },
                { 6.679546e-005, 2.374262e-004, -1.019560e-004, 6.679546e-007, 2.374262e-006, -1.019560e-006 },
                { -2.868345e-005, -1.019560e-004, 4.378217e-005, -2.868345e-007, -1.019560e-006, 4.378217e-007 }
        });

        final Frame nonInertialFrame = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        final Frame inertialFrame = FramesFactory.getGCRF();

        // When & Then
        Assertions.assertThrows(OrekitException.class,
                                () -> {
                                    new StateCovariance(randomCovarianceMatrix, initialDate, nonInertialFrame, OrbitType.CIRCULAR, PositionAngle.MEAN).changeCovarianceFrame(initialOrbit, inertialFrame);
                                });

        Assertions.assertThrows(OrekitException.class,
                                () -> {
                                	new StateCovariance(randomCovarianceMatrix, initialDate, nonInertialFrame, OrbitType.EQUINOCTIAL, PositionAngle.MEAN).changeCovarianceFrame(initialOrbit, LOFType.QSW);
                                });

    }

}
