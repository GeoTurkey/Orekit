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
import org.apache.commons.math.util.FastMath;

/** Class for fitting evolution of osculating orbital parameters.
 * <p>
 * This class is derived from a class provided by Orekit tutorials
 * and distributed under the terms of the Apache 2 License.
 * </p>
 * 
 * @author Luc Maisonobe
 */
public class SecularAndHarmonic implements ParametricUnivariateFunction {

    /** Degree of polynomial secular part. */
    private final int secularDegree;

    /** Pulsations of harmonic part. */
    private final double[] pulsations;

    /** Simple constructor.
     * @param secularDegree degree of polynomial secular part
     * @param pulsations pulsations of harmonic part
     */
    public SecularAndHarmonic(final int secularDegree, final double ... pulsations) {
        this.secularDegree = secularDegree;
        this.pulsations    = pulsations.clone();
    }

    /** {@inheritDoc} */
    public double[] gradient(double x, double ... parameters) {

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

    /** {@inheritDoc} */
    public double value(final double x, final double ... parameters) {
        return meanValue(secularDegree, pulsations.length, x, parameters);
    }

    /** Get mean value, truncated to first components.
     * @param degree degree of polynomial secular part
     * @param harmonics number of harmonics terms to consider
     * @param time time parameter
     * @param parameters models parameters (must include all parameters,
     * including the ones ignored due to model truncation)
     * @return mean value
     */
    public double meanValue(final int degree, final int harmonics,
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

}
