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
package eu.eumetsat.skat.strategies;

import org.apache.commons.math.analysis.ParametricUnivariateFunction;
import org.apache.commons.math.optimization.fitting.CurveFitter;
import org.apache.commons.math.optimization.general.LevenbergMarquardtOptimizer;
import org.apache.commons.math.util.FastMath;
import org.orekit.time.AbsoluteDate;

/** Class for fitting evolution of osculating orbital parameters.
 * <p>
 * This class is derived from a class provided by Orekit tutorials
 * and distributed under the terms of the Apache 2 License.
 * </p>
 * 
 * @author Luc Maisonobe
 */
public class SecularAndHarmonic {

    /** Degree of polynomial secular part. */
    private final int secularDegree;

    /** Pulsations of harmonic part. */
    private final double[] pulsations;

    /** Curve fitting engine. */
    private CurveFitter fitter;

    /** Reference date for the model. */
    private AbsoluteDate reference;

    /** Fitted parameters. */
    private double[] fitted;

    /** Simple constructor.
     * @param secularDegree degree of polynomial secular part
     * @param pulsations pulsations of harmonic part
     */
    public SecularAndHarmonic(final int secularDegree, final double ... pulsations) {
        this.secularDegree = secularDegree;
        this.pulsations    = pulsations.clone();
    }

    /** Reset fitting.
     * @param date reference date
     * @param initialGuess initial guess for the parameters
     * @see #getReferenceDate()
     */
    public void resetFitting(final AbsoluteDate date, final double ... initialGuess) {
        fitter    = new CurveFitter(new LevenbergMarquardtOptimizer());
        reference = date;
        fitted    = initialGuess.clone();
    }

    /** Add a fitting point.
     * @param date date of the point
     * @param osculatingValue osculating value
     */
    public void addPoint(final AbsoluteDate date, final double osculatingValue) {
        fitter.addObservedPoint(date.durationFrom(reference), osculatingValue);
    }

    /** Get the referene date.
     * @return reference date
     * @see #resetFitting(AbsoluteDate, double...)
     */
    public AbsoluteDate getReferenceDate() {
        return reference;
    }

    /** Get an upper bound of the fitted harmonic amplitude.
     * @return upper bound of the fitted harmonic amplitude
     */
    public double getHarmonicAmplitude() {
        double amplitude = 0;
        for (int i = 0; i < pulsations.length; ++i) {
            amplitude += FastMath.hypot(fitted[secularDegree + 2 * i + 1],
                                    fitted[secularDegree + 2 * i + 2]);
        }
        return amplitude;
    }

    /** Fit parameters.
     * @see #getFittedParameters()
     */
    public void fit() {

        fitted = fitter.fit(new ParametricUnivariateFunction() {

            /** {@inheritDoc} */
            public double value(double x, double... parameters) {
                return truncatedValue(secularDegree, pulsations.length, x, parameters);
            }
            
            /** {@inheritDoc} */
            public double[] gradient(double x, double... parameters) {
                final double[] gradient = new double[secularDegree + 1 + 2 * pulsations.length];

                // secular part
                double xN = 1.0;
                for (int i = 0; i <= secularDegree; ++i) {
                    gradient[i] = xN;
                    xN *= x;
                }

                // harmonic part
                for (int i = 0; i < pulsations.length; ++i) {
                    gradient[secularDegree + 2 * i + 1] = FastMath.cos(pulsations[i] * x);
                    gradient[secularDegree + 2 * i + 2] = FastMath.sin(pulsations[i] * x);
                }

                return gradient;
            }

        }, fitted);

    }

    /** Get a copy of the last fitted parameters.
     * @return copy of the last fitted parameters.
     * @see #fit()
     */
    public double[] getFittedParameters() {
        return fitted.clone();
    }

    /** Get fitted osculating value.
     * @param date current date
     * @return osculating value at current date
     */
    public double osculatingValue(final AbsoluteDate date) {
        return truncatedValue(secularDegree, pulsations.length,
                         date.durationFrom(reference), fitted);
    }

    /** Get fitted osculating derivative.
     * @param date current date
     * @return osculating derivative at current date
     */
    public double osculatingDerivative(final AbsoluteDate date) {
        return truncatedDerivative(secularDegree, pulsations.length,
                                   date.durationFrom(reference), fitted);
    }

    /** Get mean value, truncated to first components.
     * @param date current date
     * @param degree degree of polynomial secular part
     * @param harmonics number of harmonics terms to consider
     * @return mean value at current date
     */
    public double meanValue(final AbsoluteDate date, final int degree, final int harmonics) {
        return truncatedValue(degree, harmonics, date.durationFrom(reference), fitted);
    }

    /** Get mean derivative, truncated to first components.
     * @param date current date
     * @param degree degree of polynomial secular part
     * @param harmonics number of harmonics terms to consider
     * @return mean derivative at current date
     */
    public double meanDerivative(final AbsoluteDate date, final int degree, final int harmonics) {
        return truncatedDerivative(degree, harmonics, date.durationFrom(reference), fitted);
    }

    /** Get value truncated to first components.
     * @param degree degree of polynomial secular part
     * @param harmonics number of harmonics terms to consider
     * @param time time parameter
     * @param parameters models parameters (must include all parameters,
     * including the ones ignored due to model truncation)
     * @return truncated value
     */
    private double truncatedValue(final int degree, final int harmonics,
                                  final double time, final double ... parameters) {

        double value = 0;

        // secular part
        double tN = 1.0;
        for (int i = 0; i <= degree; ++i) {
            value += parameters[i] * tN;
            tN    *= time;
        }

        // harmonic part
        for (int i = 0; i < harmonics; ++i) {
            value += parameters[secularDegree + 2 * i + 1] * FastMath.cos(pulsations[i] * time) +
                     parameters[secularDegree + 2 * i + 2] * FastMath.sin(pulsations[i] * time);
        }

        return value;

    }

    /** Get derivative truncated to first components.
     * @param degree degree of polynomial secular part
     * @param harmonics number of harmonics terms to consider
     * @param time time parameter
     * @param parameters models parameters (must include all parameters,
     * including the ones ignored due to model truncation)
     * @return truncated derivative
     */
    private double truncatedDerivative(final int degree, final int harmonics,
                                       final double time, final double ... parameters) {

        double derivative = 0;

        // secular part
        double tN = 1.0;
        for (int i = 1; i <= degree; ++i) {
            derivative += i * parameters[i] * tN;
            tN    *= time;
        }

        // harmonic part
        for (int i = 0; i < harmonics; ++i) {
            derivative += pulsations[i] * (-parameters[secularDegree + 2 * i + 1] * FastMath.sin(pulsations[i] * time) +
                                            parameters[secularDegree + 2 * i + 2] * FastMath.cos(pulsations[i] * time));
        }

        return derivative;

    }

}
