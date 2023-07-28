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
package org.orekit.propagation;

import org.hipparchus.analysis.polynomials.SmoothStepFactory;
import org.hipparchus.stat.descriptive.DescriptiveStatistics;
import org.hipparchus.util.FastMath;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.orekit.Utils;
import org.orekit.data.DataContext;
import org.orekit.forces.gravity.potential.AstronomicalAmplitudeReader;
import org.orekit.forces.gravity.potential.EGMFormatReader;
import org.orekit.forces.gravity.potential.FESCHatEpsilonReader;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.OceanLoadDeformationCoefficients;
import org.orekit.frames.Frame;
import org.orekit.frames.LOFType;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitBlender;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.analytical.AbstractAnalyticalPropagator;
import org.orekit.propagation.analytical.BrouwerLyddanePropagator;
import org.orekit.propagation.analytical.EcksteinHechlerPropagator;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.time.TimeInterpolator;
import org.orekit.time.TimeStampedPair;
import org.orekit.utils.Constants;

import java.util.Map;

class StateCovarianceBlenderTest {

    private static SpacecraftState sergeiState;
    private static Orbit           sergeiOrbit;
    private static Frame           sergeiFrame;

    // Constants
    private final double DEFAULT_SERGEI_PROPAGATION_TIME   = 2400;
    private final double DEFAUTL_SERGEI_TABULATED_TIMESTEP = 2400;

    @BeforeAll
    public static void setUp() {
        Utils.setDataRoot("regular-data:potential/egm-format:atmosphere:tides:regular-data/de405-ephemerides");
        GravityFieldFactory.addPotentialCoefficientsReader(new EGMFormatReader("EGM96-truncated-21x21", true));
        AstronomicalAmplitudeReader aaReader =
                new AstronomicalAmplitudeReader("hf-fes2004.dat", 5, 2, 3, 1.0);
        DataContext.getDefault().getDataProvidersManager().feed(aaReader.getSupportedNames(), aaReader);
        Map<Integer, Double> map = aaReader.getAstronomicalAmplitudesMap();
        GravityFieldFactory.addOceanTidesReader(new FESCHatEpsilonReader("fes2004-7x7.dat",
                                                                         0.01, FastMath.toRadians(1.0),
                                                                         OceanLoadDeformationCoefficients.IERS_2010,
                                                                         map));
        // Default orbit case
        sergeiState = StateCovarianceKeplerianHermiteInterpolatorTest.generateSergeiReferenceState();
        sergeiOrbit = sergeiState.getOrbit();
        sergeiFrame = sergeiOrbit.getFrame();
    }

    private void doTestBlending(final double propagationHorizon, final double tabulatedTimeStep,
                                final SmoothStepFactory.SmoothStepFunction blendingFunction,
                                final AbstractAnalyticalPropagator analyticalPropagator,
                                final double expectedMeanRMSPositionError,
                                final double expectedMeanRMSVelocityError,
                                final double expectedMedianRMSPositionError,
                                final double expectedMedianRMSVelocityError,
                                final double expectedMaxRMSPositionError,
                                final double expectedMaxRMSVelocityError,
                                final double tolerance) {
        final TimeInterpolator<Orbit> orbitInterpolator = new OrbitBlender(blendingFunction,
                                                                           analyticalPropagator,
                                                                           sergeiFrame);

        final TimeInterpolator<TimeStampedPair<Orbit, StateCovariance>> covarianceInterpolator =
                new StateCovarianceBlender(blendingFunction, orbitInterpolator,
                                           sergeiFrame, OrbitType.CARTESIAN,
                                           PositionAngle.MEAN);

        // Create state interpolator
        final TimeInterpolator<SpacecraftState> stateInterpolator =
                new SpacecraftStateInterpolator(sergeiFrame, orbitInterpolator, null, null, null, null);

        // When
        final DescriptiveStatistics[] relativeRMSSigmaError =
                StateCovarianceKeplerianHermiteInterpolatorTest.computeStatisticsCovarianceInterpolationOnSergeiCase(
                        propagationHorizon, tabulatedTimeStep, stateInterpolator, covarianceInterpolator);

        // Then
        // Results obtained when using modified orbit date to use truncated JPL test resource file
        Assertions.assertEquals(expectedMeanRMSPositionError, relativeRMSSigmaError[0].getMean(), tolerance);
        Assertions.assertEquals(expectedMeanRMSVelocityError, relativeRMSSigmaError[1].getMean(), tolerance);
        Assertions.assertEquals(expectedMedianRMSPositionError, relativeRMSSigmaError[0].getPercentile(50), tolerance);
        Assertions.assertEquals(expectedMedianRMSVelocityError, relativeRMSSigmaError[1].getPercentile(50), tolerance);
        Assertions.assertEquals(expectedMaxRMSPositionError, relativeRMSSigmaError[0].getMax(), tolerance);
        Assertions.assertEquals(expectedMaxRMSVelocityError, relativeRMSSigmaError[1].getMax(), tolerance);
    }

    /**
     * Test based on the full force model test case from TANYGIN, Sergei. Efficient covariance interpolation using blending
     * of approximate covariance propagations. The Journal of the Astronautical Sciences, 2014, vol. 61, no 1, p. 107-132.
     * <p>
     * However, note that the exact result is not known and only an approximated value can be deduced from the available
     * graph in the aforementioned paper. That is why it is both a validation test and a non regression test.
     * <p>
     * This instance of the test aims to test the Quadratic Blending method with standard Keplerian dynamics.
     */
    @Test
    @DisplayName("test Keplerian quadratic blending interpolation on full force model test case from: "
            + "TANYGIN, Sergei. Efficient covariance interpolation using blending of approximate covariance propagations. "
            + "The Journal of the Astronautical Sciences, 2014, vol. 61, no 1, p. 107-132.")
    void testKeplerianQuadraticBlending() {
        // Given
        // Create state covariance interpolator
        final SmoothStepFactory.SmoothStepFunction blendingFunction = SmoothStepFactory.getQuadratic();

        // When & Then
        doTestBlending(DEFAULT_SERGEI_PROPAGATION_TIME, DEFAUTL_SERGEI_TABULATED_TIMESTEP, blendingFunction,
                       new KeplerianPropagator(sergeiOrbit),
                       0.10893117502278186,
                       0.2596446611452972,
                       0.10519888609039357,
                       0.28036786120486396,
                       0.29692889227737684,
                       0.5750518125401441, 1e-16);

        // Results obtained when using Sergei reference date
/*        Assertions.assertEquals(0.12086964077199346, relativeRMSSigmaError[0].getMean(), 1e-17);
        Assertions.assertEquals(0.21871712884727984, relativeRMSSigmaError[1].getMean(), 1e-17);
        Assertions.assertEquals(0.12172900928265865, relativeRMSSigmaError[0].getPercentile(50), 1e-17);
        Assertions.assertEquals(0.24456923477457276, relativeRMSSigmaError[1].getPercentile(50), 1e-17);
        Assertions.assertEquals(0.24004509397020612, relativeRMSSigmaError[0].getMax(), 1e-17);
        Assertions.assertEquals(0.3375639988469634, relativeRMSSigmaError[1].getMax(), 1e-16);*/
    }

    /**
     * Test based on the full force model test case from TANYGIN, Sergei. Efficient covariance interpolation using blending
     * of approximate covariance propagations. The Journal of the Astronautical Sciences, 2014, vol. 61, no 1, p. 107-132.
     * <p>
     * This instance of the test aims to test the Quadratic Blending method using Brouwer Lyddane propagation. This is a
     * non-regression test.
     */
    @Test
    @DisplayName("test Brouwer Lyddane quadratic blending interpolation on full force model test case from: "
            + "TANYGIN, Sergei. Efficient covariance interpolation using blending of approximate covariance propagations. "
            + "The Journal of the Astronautical Sciences, 2014, vol. 61, no 1, p. 107-132.")
    void testBrouwerLyddaneQuadraticBlending() {
        // Given
        // Create state covariance interpolator
        final SmoothStepFactory.SmoothStepFunction blendingFunction = SmoothStepFactory.getQuadratic();

        final AbstractAnalyticalPropagator propagator = new BrouwerLyddanePropagator(sergeiOrbit,
                                                                                     sergeiState.getMass(),
                                                                                     Constants.EIGEN5C_EARTH_EQUATORIAL_RADIUS,
                                                                                     Constants.EIGEN5C_EARTH_MU,
                                                                                     Constants.EIGEN5C_EARTH_C20,
                                                                                     Constants.EIGEN5C_EARTH_C30,
                                                                                     Constants.EIGEN5C_EARTH_C40,
                                                                                     Constants.EIGEN5C_EARTH_C50,
                                                                                     0);

        // When & Then
        doTestBlending(DEFAULT_SERGEI_PROPAGATION_TIME, DEFAUTL_SERGEI_TABULATED_TIMESTEP, blendingFunction,
                       propagator,
                       0.08986015483172992,
                       0.22449764469077402,
                       0.09705016641506019,
                       0.22462741715597975,
                       0.15788797289350096,
                       0.5635732855379268, 1e-16);

        // Results obtained when using Sergei reference date
/*        Assertions.assertEquals(0.07645785479359624, relativeRMSSigmaError[0].getMean(), 1e-17);
        Assertions.assertEquals(0.17941792898602038, relativeRMSSigmaError[1].getMean(), 1e-17);
        Assertions.assertEquals(0.08259069655149026, relativeRMSSigmaError[0].getPercentile(50), 1e-17);
        Assertions.assertEquals(0.18352413417267063, relativeRMSSigmaError[1].getPercentile(50), 1e-17);
        Assertions.assertEquals(0.13164670404592496, relativeRMSSigmaError[0].getMax(), 1e-17);
        Assertions.assertEquals(0.32564919981018114, relativeRMSSigmaError[1].getMax(), 1e-16);*/
    }

    /**
     * Test based on the full force model test case from TANYGIN, Sergei. Efficient covariance interpolation using blending
     * of approximate covariance propagations. The Journal of the Astronautical Sciences, 2014, vol. 61, no 1, p. 107-132.
     * <p>
     * This instance of the test aims to test the Quadratic Blending method using Eckstein Hechler propagation. This is a
     * non-regression test.
     */
    @Test
    @DisplayName("test Ekstein Hechler quadratic blending interpolation on full force model test case from: "
            + "TANYGIN, Sergei. Efficient covariance interpolation using blending of approximate covariance propagations. "
            + "The Journal of the Astronautical Sciences, 2014, vol. 61, no 1, p. 107-132.")
    void testEksteinHechlerQuadraticBlending() {
        // Given
        // Create state covariance interpolator
        final SmoothStepFactory.SmoothStepFunction blendingFunction = SmoothStepFactory.getQuadratic();

        final AbstractAnalyticalPropagator propagator = new EcksteinHechlerPropagator(sergeiOrbit,
                                                                                      sergeiState.getMass(),
                                                                                      Constants.EIGEN5C_EARTH_EQUATORIAL_RADIUS,
                                                                                      Constants.EIGEN5C_EARTH_MU,
                                                                                      Constants.EIGEN5C_EARTH_C20,
                                                                                      Constants.EIGEN5C_EARTH_C30,
                                                                                      Constants.EIGEN5C_EARTH_C40,
                                                                                      Constants.EIGEN5C_EARTH_C50,
                                                                                      Constants.EIGEN5C_EARTH_C60);

        // When & Then
        doTestBlending(DEFAULT_SERGEI_PROPAGATION_TIME, DEFAUTL_SERGEI_TABULATED_TIMESTEP, blendingFunction,
                       propagator,
                       0.09919050025473311,
                       0.2032481714771728,
                       0.09969057639046483,
                       0.21519642371336853,
                       0.18189925769828325,
                       0.5453231160284159, 1e-16);

        // Results obtained when using Sergei reference date
/*        Assertions.assertEquals(0.08688580997030507, relativeRMSSigmaError[0].getMean(), 1e-17);
        Assertions.assertEquals(0.15630296926592135, relativeRMSSigmaError[1].getMean(), 1e-17);
        Assertions.assertEquals(0.07771211628338093, relativeRMSSigmaError[0].getPercentile(50), 1e-17);
        Assertions.assertEquals(0.17056867629454775, relativeRMSSigmaError[1].getPercentile(50), 1e-17);
        Assertions.assertEquals(0.16747846953637413, relativeRMSSigmaError[0].getMax(), 1e-17);
        Assertions.assertEquals(0.26557409638784585, relativeRMSSigmaError[1].getMax(), 1e-16);*/
    }

    /**
     * Test based on the full force model test case from TANYGIN, Sergei. Efficient covariance interpolation using blending
     * of approximate covariance propagations. The Journal of the Astronautical Sciences, 2014, vol. 61, no 1, p. 107-132.
     * <p>
     * This instance of the test is a non regression test aiming to test the results obtained when interpolating in a local
     * orbital frame.
     * <p>
     * Bad results are seen regarding velocity interpolation. This has been checked to be independent of the method used to
     * interpolate the state covariance. Moreover, the same results are achieved if we interpolate in an inertial frame
     * (which has been seen to give very good results as in {@link #testKeplerianQuadraticBlending()}),and then express it in
     * a non-inertial local orbital frame. It has also been verified that it was not (mainly) due to errors in orbit
     * interpolation as even using exact orbit for frame conversion would only slightly improve the results (<0.1%
     * improvements). Hence, the only explanation found is a sensitivity issue linked to non-inertial local orbital frame.
     */
    @Test
    @DisplayName("test Keplerian quadratic blending expressed in LOF on full force model test case from : "
            + "TANYGIN, Sergei. Efficient covariance interpolation using blending of approximate covariance propagations. "
            + "The Journal of the Astronautical Sciences, 2014, vol. 61, no 1, p. 107-132.")
    void testLOFKeplerianBlending() {
        // Given
        // Create state covariance interpolator
        final SmoothStepFactory.SmoothStepFunction blendingFunction = SmoothStepFactory.getQuadratic();

        final TimeInterpolator<Orbit> orbitInterpolator = new OrbitBlender(blendingFunction,
                                                                           new KeplerianPropagator(sergeiOrbit),
                                                                           sergeiFrame);

        final LOFType DEFAULT_LOFTYPE = LOFType.TNW;
        final AbstractStateCovarianceInterpolator covarianceInterpolator =
                new StateCovarianceBlender(blendingFunction, orbitInterpolator, DEFAULT_LOFTYPE);

        // When
        final DescriptiveStatistics[] relativeRMSSigmaError =
                StateCovarianceKeplerianHermiteInterpolatorTest.computeStatisticsCovarianceLOFInterpolation(
                        DEFAULT_SERGEI_PROPAGATION_TIME, DEFAUTL_SERGEI_TABULATED_TIMESTEP,
                        DEFAULT_LOFTYPE, covarianceInterpolator);

        // Then
        // Results obtained when using modified orbit date to use truncated JPL test resource file
        Assertions.assertEquals(0.12191976038569027, relativeRMSSigmaError[0].getMean(), 1e-22);
        Assertions.assertEquals(9.046997825346464, relativeRMSSigmaError[1].getMean(), 1e-21);
        Assertions.assertEquals(0.12687141787871853, relativeRMSSigmaError[0].getPercentile(50), 1e-22);
        Assertions.assertEquals(8.849625530916887, relativeRMSSigmaError[1].getPercentile(50), 1e-21);
        Assertions.assertEquals(0.23947876206895072, relativeRMSSigmaError[0].getMax(), 1e-20);
        Assertions.assertEquals(21.469525455933358, relativeRMSSigmaError[1].getMax(), 1e-20);

        // Assert getters as well
        Assertions.assertNull(covarianceInterpolator.getOutFrame());
        Assertions.assertEquals(DEFAULT_LOFTYPE, covarianceInterpolator.getOutLOF());
        Assertions.assertEquals(OrbitType.CARTESIAN, covarianceInterpolator.getOutOrbitType());
        Assertions.assertEquals(PositionAngle.MEAN, covarianceInterpolator.getOutPositionAngle());
        Assertions.assertEquals(orbitInterpolator, covarianceInterpolator.getOrbitInterpolator());

    }

}
