/* Contributed in the public domain.
 * Licensed to CS Group (CS) under one or more
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
package org.orekit.models.earth;

import org.orekit.models.earth.GeoMagneticFieldFactory.FieldModel;

/**
 * Methods for obtaining geomagnetic fields.
 *
 * @author Evan Ward
 * @author Thomas Neidhart
 * @see GeoMagneticFieldFactory
 * @see LazyLoadedGeoMagneticFields
 * @since 10.1
 */
public interface GeoMagneticFields {

    /**
     * Get the {@link GeoMagneticField} for the given model type and year.
     *
     * @param type the field model type
     * @param year the decimal year
     * @return a {@link GeoMagneticField} for the given year and model
     * @see GeoMagneticField#getDecimalYear(int, int, int)
     */
    GeoMagneticField getField(FieldModel type, double year);

    /**
     * Get the IGRF model for the given year.
     *
     * @param year the decimal year
     * @return a {@link GeoMagneticField} for the given year
     * @see GeoMagneticField#getDecimalYear(int, int, int)
     */
    GeoMagneticField getIGRF(double year);

    /**
     * Get the WMM model for the given year.
     *
     * @param year the decimal year
     * @return a {@link GeoMagneticField} for the given year
     * @see GeoMagneticField#getDecimalYear(int, int, int)
     */
    GeoMagneticField getWMM(double year);

}