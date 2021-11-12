/* Copyright 2002-2021 CS GROUP
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
package org.orekit.forces.maneuvers.trigger;

import java.util.Arrays;
import java.util.List;

import org.hipparchus.CalculusFieldElement;
import org.hipparchus.Field;
import org.orekit.propagation.events.FieldAbstractDetector;
import org.orekit.propagation.events.FieldEventDetector;
import org.orekit.propagation.events.FieldParameterDrivenDateIntervalDetector;
import org.orekit.propagation.events.ParameterDrivenDateIntervalDetector;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.ParameterDriver;

/** Maneuver triggers based on a start and end date, with no parameter drivers.
 * @author Maxime Journot
 * @since 10.2
 */
public class DateBasedManeuverTriggers extends IntervalEventTrigger<ParameterDrivenDateIntervalDetector> {

    /** Default name for trigger. */
    public static final String DEFAULT_NAME = "";

    /** Name of the trigger (used as prefix for start and stop parameters drivers). */
    private final String name;

    /** Simple constructor.
     * @param date start (or end) data of the maneuver
     * @param duration maneuver duration (if positive, maneuver is from date to date + duration,
     * if negative, maneuver will be from date - duration to date)
     */
    public DateBasedManeuverTriggers(final AbsoluteDate date, final double duration) {
        this(DEFAULT_NAME, date, duration);
    }

    /** Simple constructor.
     * @param name name of the trigger (used as prefix for start and stop parameters drivers)
     * @param date start (or end) data of the maneuver
     * @param duration maneuver duration (if positive, maneuver is from date to date + duration,
     * if negative, maneuver will be from date - duration to date)
     * @since 11.1
     */
    public DateBasedManeuverTriggers(final String name, final AbsoluteDate date, final double duration) {
        super(createDetector(name, date, duration));
        this.name = name;
    }

    /** Create a date detector from one boundary and signed duration.
     * @param prefix for start and stop parameters drivers
     * @param date start (or end) data of the maneuver
     * @param duration maneuver duration (if positive, maneuver is from date to date + duration,
     * if negative, maneuver will be from date - duration to date)
     * @return date detector
     * @since 11.1
     */
    private static ParameterDrivenDateIntervalDetector createDetector(final String prefix, final AbsoluteDate date, final double duration) {
        if (duration >= 0) {
            return new ParameterDrivenDateIntervalDetector(prefix, date, date.shiftedBy(duration));
        } else {
            return new ParameterDrivenDateIntervalDetector(prefix, date.shiftedBy(duration), date);
        }
    }

    /** {@inheritDoc} */
    @Override
    public String getName() {
        return name;
    }

    /** Get the start date.
     * @return the start date
     */
    public AbsoluteDate getStartDate() {
        return getFiringIntervalDetector().getStartDriver().getDate();
    }

    /** Get the end date.
     * @return the end date
     */
    public AbsoluteDate getEndDate() {
        return getFiringIntervalDetector().getStopDriver().getDate();
    }

    /** Get the duration of the maneuver (s).
     * duration = endDate - startDate
     * @return the duration of the maneuver (s)
     */
    public double getDuration() {
        return getEndDate().durationFrom(getStartDate());
    }

    /** {@inheritDoc} */
    @Override
    protected <D extends FieldEventDetector<S>, S extends CalculusFieldElement<S>>
        FieldAbstractDetector<D, S> convertIntervalDetector(final Field<S> field, final ParameterDrivenDateIntervalDetector detector) {

        final FieldParameterDrivenDateIntervalDetector<S> fd =
                        new FieldParameterDrivenDateIntervalDetector<S>(field, "",
                                                                        detector.getStartDriver().getBaseDate(),
                                                                        detector.getStopDriver().getBaseDate());
        fd.getStartDriver().setName(detector.getStartDriver().getName());
        fd.getStopDriver().setName(detector.getStopDriver().getName());

        @SuppressWarnings("unchecked")
        final FieldAbstractDetector<D, S> converted = (FieldAbstractDetector<D, S>) fd;
        return converted;

    }

    /** {@inheritDoc} */
    @Override
    public List<ParameterDriver> getParametersDrivers() {
        return Arrays.asList(getFiringIntervalDetector().getStartDriver(),
                             getFiringIntervalDetector().getStopDriver());
    }

}
