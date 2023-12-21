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
package org.orekit.models.earth.troposphere;

import org.hipparchus.CalculusFieldElement;
import org.hipparchus.Field;
import org.hipparchus.util.MathArrays;
import org.orekit.annotation.DefaultDataContext;
import org.orekit.bodies.FieldGeodeticPoint;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataContext;
import org.orekit.models.earth.weather.FieldPressureTemperatureHumidity;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.FieldAbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.utils.FieldTrackingCoordinates;
import org.orekit.utils.TrackingCoordinates;

/** The Vienna1 tropospheric delay model for radio techniques.
 * The Vienna model data are given with a time interval of 6 hours
 * as well as on a global 2.5° * 2.0° grid.
 *
 * This version considered the height correction for the hydrostatic part
 * developed by Niell, 1996.
 *
 * @see "Boehm, J., Werl, B., and Schuh, H., (2006),
 *       Troposhere mapping functions for GPS and very long baseline
 *       interferometry from European Centre for Medium-Range Weather
 *       Forecasts operational analysis data, J. Geophy. Res., Vol. 111,
 *       B02406, doi:10.1029/2005JB003629"
 *
 * @author Bryan Cazabonne
 * @deprecated as of 12.1, replaced by {@link ViennaOne}
 */
@Deprecated
public class ViennaOneModel extends ViennaOne implements DiscreteTroposphericModel, MappingFunction {

    /** Values of hydrostatic and wet delays as provided by the Vienna model. */
    private final double[] zenithDelay;

    /** Build a new instance.
     *
     * <p>This constructor uses the {@link DataContext#getDefault() default data context}.
     *
     * @param coefficientA The a coefficients for the computation of the wet and hydrostatic mapping functions.
     * @param zenithDelay Values of hydrostatic and wet delays
     * @see #ViennaOneModel(double[], double[], TimeScale)
     */
    @DefaultDataContext
    public ViennaOneModel(final double[] coefficientA, final double[] zenithDelay) {
        this(coefficientA, zenithDelay,
             DataContext.getDefault().getTimeScales().getUTC());
    }

    /**
     * Build a new instance.
     *
     * @param coefficientA The a coefficients for the computation of the wet and
     *                     hydrostatic mapping functions.
     * @param zenithDelay  Values of hydrostatic and wet delays
     * @param utc          UTC time scale.
     * @since 10.1
     */
    public ViennaOneModel(final double[] coefficientA,
                          final double[] zenithDelay,
                          final TimeScale utc) {
        super(new ConstantViennaAProvider(new ViennaACoefficients(coefficientA[0], coefficientA[1])),
              new ConstantTroposphericModel(new TroposphericDelay(zenithDelay[0], zenithDelay[1],
                                                                  zenithDelay[0], zenithDelay[1])),
              utc);
        this.zenithDelay = zenithDelay.clone();
    }

    /** {@inheritDoc} */
    @Override
    @Deprecated
    public double pathDelay(final double elevation, final GeodeticPoint point,
                            final double[] parameters, final AbsoluteDate date) {
        return pathDelay(new TrackingCoordinates(0.0, elevation, 0.0),
                         point, TroposphericModelUtils.STANDARD_ATMOSPHERE, parameters, date).
               getDelay();
    }

    /** {@inheritDoc} */
    @Override
    @Deprecated
    public <T extends CalculusFieldElement<T>> T pathDelay(final T elevation, final FieldGeodeticPoint<T> point,
                                                           final T[] parameters, final FieldAbsoluteDate<T> date) {
        return pathDelay(new FieldTrackingCoordinates<>(date.getField().getZero(), elevation, date.getField().getZero()),
                         point,
                         new FieldPressureTemperatureHumidity<>(date.getField(), TroposphericModelUtils.STANDARD_ATMOSPHERE),
                         parameters, date).
               getDelay();
    }

    /** This method allows the  computation of the zenith hydrostatic and
     * zenith wet delay. The resulting element is an array having the following form:
     * <ul>
     * <li>T[0] = D<sub>hz</sub> → zenith hydrostatic delay
     * <li>T[1] = D<sub>wz</sub> → zenith wet delay
     * </ul>
     * @param point station location
     * @param parameters tropospheric model parameters
     * @param date current date
     * @return a two components array containing the zenith hydrostatic and wet delays.
     */
    public double[] computeZenithDelay(final GeodeticPoint point, final double[] parameters, final AbsoluteDate date) {
        return zenithDelay.clone();
    }

    /** This method allows the  computation of the zenith hydrostatic and
     * zenith wet delay. The resulting element is an array having the following form:
     * <ul>
     * <li>T[0] = D<sub>hz</sub> → zenith hydrostatic delay
     * <li>T[1] = D<sub>wz</sub> → zenith wet delay
     * </ul>
     * @param <T> type of the elements
     * @param point station location
     * @param parameters tropospheric model parameters
     * @param date current date
     * @return a two components array containing the zenith hydrostatic and wet delays.
     */
    public <T extends CalculusFieldElement<T>> T[] computeZenithDelay(final FieldGeodeticPoint<T> point, final T[] parameters,
                                                                      final FieldAbsoluteDate<T> date) {
        final Field<T> field = date.getField();
        final T zero = field.getZero();
        final T[] delays = MathArrays.buildArray(field, 2);
        delays[0] = zero.newInstance(zenithDelay[0]);
        delays[1] = zero.newInstance(zenithDelay[1]);
        return delays;
    }

    /** {@inheritDoc} */
    @Override
    @Deprecated
    public double[] mappingFactors(final double elevation, final GeodeticPoint point,
                                   final AbsoluteDate date) {
        return mappingFactors(new TrackingCoordinates(0.0, elevation, 0.0),
                              point,
                              TroposphericModelUtils.STANDARD_ATMOSPHERE,
                              date);
    }

    /** {@inheritDoc} */
    @Override
    @Deprecated
    public <T extends CalculusFieldElement<T>> T[] mappingFactors(final T elevation, final FieldGeodeticPoint<T> point,
                                                                  final FieldAbsoluteDate<T> date) {
        return mappingFactors(new FieldTrackingCoordinates<>(date.getField().getZero(), elevation, date.getField().getZero()),
                              point,
                              new FieldPressureTemperatureHumidity<>(date.getField(),
                                                                     TroposphericModelUtils.STANDARD_ATMOSPHERE),
                              date);
    }

}
