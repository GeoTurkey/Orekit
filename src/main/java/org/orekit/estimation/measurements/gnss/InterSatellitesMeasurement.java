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
package org.orekit.estimation.measurements.gnss;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Function;
import java.util.function.ToDoubleFunction;

import org.hipparchus.analysis.differentiation.Gradient;
import org.orekit.estimation.measurements.ObservableSatellite;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.FieldAbsoluteDate;
import org.orekit.utils.FieldPVCoordinatesProvider;
import org.orekit.utils.PVCoordinatesProvider;
import org.orekit.utils.ShiftingPVCoordinatesProvider;
import org.orekit.utils.TimeStampedFieldPVCoordinates;

/** BAse class for measurement between two satellites that are both estimated.
 * <p>
 * The measurement is considered to be a signal emitted from
 * a remote satellite and received by a local satellite.
 * Its value is the number of cycles between emission and reception.
 * The motion of both spacecraft during the signal flight time
 * are taken into account. The date of the measurement corresponds to the
 * reception on ground of the emitted signal.
 * </p>
 * @param <T> type of the measurement
 * @author Luc Maisonobe
 * @since 12.1
 */
public abstract class InterSatellitesMeasurement<T extends ObservedMeasurement<T>> extends AbstractOnBoardMeasurement<T> {

    /** Constructor.
     * @param date date of the measurement
     * @param observed observed value
     * @param sigma theoretical standard deviation
     * @param baseWeight base weight
     * @param local satellite which receives the signal and performs the measurement
     * @param remote remote satellite which simply emits the signal
     */
    public InterSatellitesMeasurement(final AbsoluteDate date, final double observed,
                                      final double sigma, final double baseWeight,
                                      final ObservableSatellite local,
                                      final ObservableSatellite remote) {
        // Call to super constructor
        super(date, observed, sigma, baseWeight, Arrays.asList(local, remote));

        addParameterDriver(local.getClockOffsetDriver());
        addParameterDriver(local.getClockDriftDriver());
        addParameterDriver(local.getClockAccelerationDriver());
        addParameterDriver(remote.getClockOffsetDriver());
        addParameterDriver(remote.getClockDriftDriver());
        addParameterDriver(remote.getClockAccelerationDriver());

    }

    /** {@inheritDoc} */
    @Override
    protected PVCoordinatesProvider getDoubleRemotePV(final SpacecraftState[] states) {
        return new ShiftingPVCoordinatesProvider(states[1].getPVCoordinates(), states[1].getFrame());
    }

    /** {@inheritDoc} */
    @Override
    protected ToDoubleFunction<AbsoluteDate> getDoubleRemoteClock() {
        return getSatellites().get(1).getQuadraticClockModel(getDate())::getOffset;
    }

    /** {@inheritDoc} */
    @Override
    protected FieldPVCoordinatesProvider<Gradient> getGradientRemotePV(final SpacecraftState[] states,
                                                                       final int freeParameters) {
        // convert the SpacecraftState to a FieldPVCoordinatesProvider<Gradient>
        return (date, frame) -> {

            // shift the raw (no derivatives) remote state
            final AbsoluteDate    dateBase = date.toAbsoluteDate();
            final SpacecraftState shifted  = states[1].shiftedBy(dateBase.durationFrom(states[1]));

            // set up the derivatives with respect to remote state
            final TimeStampedFieldPVCoordinates<Gradient> pv = getCoordinates(shifted, 6, freeParameters);

            // add remaining derivatives, using a trick: we shift the date by 0, with derivatives
            final Gradient zeroWithDerivatives = date.durationFrom(dateBase);
            return pv.shiftedBy(zeroWithDerivatives);

        };
    }

    /** {@inheritDoc} */
    @Override
    protected Function<FieldAbsoluteDate<Gradient>, Gradient> getGradientRemoteClock(final int freeParameters,
                                                                                     final Map<String, Integer> indices) {
        return getSatellites().get(1).getQuadraticClockModel(freeParameters, indices, getDate())::getOffset;
   }

}
