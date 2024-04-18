/* Copyright 2022-2024 Romain Serra
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
package org.orekit.propagation.events.intervals;

import org.orekit.propagation.events.AdaptableInterval;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeStamped;

/**
 * Factory class for {@link AdaptableInterval} suitable for date detection.
 * @see AdaptableInterval
 * @see org.orekit.propagation.events.DateDetector
 * @author Romain Serra
 * @since 12.1
 */
public class DateDetectionAdaptableIntervalFactory {

    /**
     * Private constructor.
     */
    private DateDetectionAdaptableIntervalFactory() {
        // factory class
    }

    /**
     * Method providing a candidate {@link AdaptableInterval} for date(s) detection with forward propagation.
     * @param timeStampedArray dated objects to compare with
     * @return adaptable interval for forward date detection
     */
    public static AdaptableInterval getForwardDateDetectionInterval(final TimeStamped... timeStampedArray) {
        return state -> computeShortestDurationToFutureDates(state.getDate(), timeStampedArray);
    }

    /**
     * Method computing the shortest duration between a reference and other dated objects, only selecting future ones.
     * @param reference reference defining future
     * @param timeStampedArray dated objects to measure duration to
     * @return duration to the closest date in the future
     */
    public static double computeShortestDurationToFutureDates(final TimeStamped reference,
                                                              final TimeStamped... timeStampedArray) {
        if (timeStampedArray.length == 0) {
            return Double.POSITIVE_INFINITY;
        } else {
            final AbsoluteDate referenceDate = reference.getDate();
            AbsoluteDate closestDate = AbsoluteDate.FUTURE_INFINITY;
            for (final TimeStamped timeStamped : timeStampedArray) {
                final AbsoluteDate date = timeStamped.getDate();
                if (date.isAfterOrEqualTo(referenceDate) && date.isBefore(closestDate)) {
                    closestDate = date;
                }
            }
            return closestDate.durationFrom(referenceDate);
        }
    }

    /**
     * Method providing a candidate {@link AdaptableInterval} for date(s) detection with backward propagation.
     * @param timeStampedArray dated objects to compare with
     * @return adaptable interval for backward date detection
     */
    public static AdaptableInterval getBackwardDateDetectionInterval(final TimeStamped... timeStampedArray) {
        return state -> computeShortestDurationFromPastDates(state.getDate(), timeStampedArray);
    }

    /**
     * Method computing the shortest duration between a reference and other dated objects, only selecting past ones.
     * @param reference reference defining past
     * @param timeStampedArray dated objects to measure duration from
     * @return duration from the closest date in the past
     */
    public static double computeShortestDurationFromPastDates(final TimeStamped reference,
                                                              final TimeStamped... timeStampedArray) {
        if (timeStampedArray.length == 0) {
            return Double.POSITIVE_INFINITY;
        } else {
            final AbsoluteDate referenceDate = reference.getDate();
            AbsoluteDate closestDate = AbsoluteDate.PAST_INFINITY;
            for (final TimeStamped timeStamped : timeStampedArray) {
                final AbsoluteDate date = timeStamped.getDate();
                if (date.isBeforeOrEqualTo(referenceDate) && date.isAfter(closestDate)) {
                    closestDate = date;
                }
            }
            return referenceDate.durationFrom(closestDate);
        }
    }
}
