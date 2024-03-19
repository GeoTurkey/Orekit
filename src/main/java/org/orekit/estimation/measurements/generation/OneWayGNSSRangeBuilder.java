/* Copyright 2002-2024 Luc Maisonobe
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
package org.orekit.estimation.measurements.generation;

import java.util.Map;

import org.hipparchus.random.CorrelatedRandomVectorGenerator;
import org.orekit.estimation.measurements.ObservableSatellite;
import org.orekit.estimation.measurements.gnss.OneWayGNSSRange;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;

/** Builder for {@link OneWayGNSSRange} measurements.
 * @author Luc Maisonobe
 * @since 12.0
 */
public class OneWayGNSSRangeBuilder extends AbstractMeasurementBuilder<OneWayGNSSRange> {

    /** Satellite which simply emits the signal. */
    private final ObservableSatellite remote;

   /** Simple constructor.
     * @param noiseSource noise source, may be null for generating perfect measurements
     * @param local satellite which receives the signal and performs the measurement
     * @param remote satellite which simply emits the signal
     * @param sigma theoretical standard deviation
     * @param baseWeight base weight
     */
    public OneWayGNSSRangeBuilder(final CorrelatedRandomVectorGenerator noiseSource,
                                  final ObservableSatellite local, final ObservableSatellite remote,
                                  final double sigma, final double baseWeight) {
        super(noiseSource, sigma, baseWeight, local, remote);
        this.remote           = remote;
    }

    /** {@inheritDoc} */
    @Override
    protected OneWayGNSSRange buildObserved(final AbsoluteDate date,
                                            final Map<ObservableSatellite, OrekitStepInterpolator> interpolators) {
        return new OneWayGNSSRange(interpolators.get(remote),
                                   remote.getQuadraticClockModel(),
                                   date, Double.NaN,
                                   getTheoreticalStandardDeviation()[0],
                                   getBaseWeight()[0], getSatellites()[0]);
    }

}
