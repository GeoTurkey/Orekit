/* Copyright 2023 Luc Maisonobe
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

package org.orekit.files.ccsds.ndm.adm.acm;

import java.util.Collections;
import java.util.List;

/** Covariance history.
 * @author Luc Maisonobe
 * @since 12.0
 */
public class AttitudeCovarianceHistory {

    /** Metadata. */
    private final AttitudeCovarianceHistoryMetadata metadata;

    /** Covariance. */
    private final List<AttitudeCovariance> covariances;

    /** Simple constructor.
     * @param metadata metadata
     * @param covariances covariances
     */
    public AttitudeCovarianceHistory(final AttitudeCovarianceHistoryMetadata metadata,
                             final List<AttitudeCovariance> covariances) {
        this.metadata    = metadata;
        this.covariances = covariances;
    }

    /** Get metadata.
     * @return metadata
     */
    public AttitudeCovarianceHistoryMetadata getMetadata() {
        return metadata;
    }

    /** Get the covariances.
     * @return covariances
     */
    public List<AttitudeCovariance> getCovariances() {
        return Collections.unmodifiableList(covariances);
    }

}
