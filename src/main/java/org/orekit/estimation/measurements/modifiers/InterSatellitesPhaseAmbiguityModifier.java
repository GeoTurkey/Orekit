/* Copyright 2002-2024 CS GROUP
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
package org.orekit.estimation.measurements.modifiers;

import java.util.List;

import org.orekit.estimation.measurements.EstimatedMeasurement;
import org.orekit.estimation.measurements.EstimatedMeasurementBase;
import org.orekit.estimation.measurements.EstimationModifier;
import org.orekit.estimation.measurements.gnss.InterSatellitesPhase;
import org.orekit.utils.ParameterDriver;

/** Class modifying theoretical inter-satellites phase measurement with ambiguity.
 * @author Bryan Cazabonne
 * @since 10.3
 */
public class InterSatellitesPhaseAmbiguityModifier extends AbstractAmbiguityModifier implements EstimationModifier<InterSatellitesPhase> {

    /** Constructor.
     * <p>
     * It is expected that many different ambiguities will be used at the
     * same time during an orbit determination, therefore they are keyed
     * using a simple integer. All ambiguities using the same key will
     * be enforced to be equal. It is the responsibility of the caller to
     * use a proper counter to manage the ambiguities properly.
     * </p>
     * @param key key to identify the ambiguity
     * @param ambiguity initial value of ambiguity
     */
    public InterSatellitesPhaseAmbiguityModifier(final int key, final double ambiguity) {
        super(key, ambiguity);
    }

    /** {@inheritDoc} */
    @Override
    public List<ParameterDriver> getParametersDrivers() {
        return getDrivers();
    }

    /** {@inheritDoc} */
    @Override
    public void modifyWithoutDerivatives(final EstimatedMeasurementBase<InterSatellitesPhase> estimated) {
        doModifyWithoutDerivatives(this, estimated);
    }

    /** {@inheritDoc} */
    @Override
    public void modify(final EstimatedMeasurement<InterSatellitesPhase> estimated) {
        doModify(this, estimated);
    }

}
