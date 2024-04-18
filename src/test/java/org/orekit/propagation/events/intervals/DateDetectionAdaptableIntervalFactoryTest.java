package org.orekit.propagation.events.intervals;

import org.hipparchus.util.FastMath;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.events.AdaptableInterval;
import org.orekit.propagation.events.DateDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.events.handlers.StopOnEvent;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeStamped;
import org.orekit.utils.Constants;

class DateDetectionAdaptableIntervalFactoryTest {

    @Test
    void testComputeShortestDurationToFutureDatesWithoutDate() {
        // GIVEN
        final AbsoluteDate date = AbsoluteDate.ARBITRARY_EPOCH;
        // WHEN
        final double actualResult = DateDetectionAdaptableIntervalFactory.computeShortestDurationToFutureDates(date);
        // THEN
        final double expectedResult = Double.POSITIVE_INFINITY;
        Assertions.assertEquals(expectedResult, actualResult);
    }

    @Test
    void testComputeShortestDurationToFutureDatesWithOneFutureDate() {
        // GIVEN
        final AbsoluteDate date = AbsoluteDate.ARBITRARY_EPOCH;
        final double expectedResult = 100;
        final AbsoluteDate shiftedDate = date.shiftedBy(expectedResult);
        // WHEN
        final double actualResult = DateDetectionAdaptableIntervalFactory.computeShortestDurationToFutureDates(date,
                shiftedDate);
        // THEN
        Assertions.assertEquals(expectedResult, actualResult);
    }

    @Test
    void testComputeShortestDurationToFutureDatesWithOnePastDate() {
        // GIVEN
        final AbsoluteDate date = AbsoluteDate.ARBITRARY_EPOCH;
        final AbsoluteDate shiftedDate = date.shiftedBy(-10.);
        // WHEN
        final double actualResult = DateDetectionAdaptableIntervalFactory.computeShortestDurationToFutureDates(date,
                shiftedDate);
        // THEN
        final double expectedResult = Double.POSITIVE_INFINITY;
        Assertions.assertEquals(expectedResult, actualResult);
    }

    @Test
    void testComputeShortestDurationFromPasDatesWithoutDate() {
        // GIVEN
        final AbsoluteDate date = AbsoluteDate.ARBITRARY_EPOCH;
        // WHEN
        final double actualResult = DateDetectionAdaptableIntervalFactory.computeShortestDurationFromPastDates(date);
        // THEN
        final double expectedResult = Double.POSITIVE_INFINITY;
        Assertions.assertEquals(expectedResult, actualResult);
    }

    @Test
    void testComputeShortestDurationFromPastDatesWithOneFutureDate() {
        // GIVEN
        final AbsoluteDate date = AbsoluteDate.ARBITRARY_EPOCH;
        final AbsoluteDate shiftedDate = date.shiftedBy(10.);
        // WHEN
        final double actualResult = DateDetectionAdaptableIntervalFactory.computeShortestDurationFromPastDates(date,
                shiftedDate);
        // THEN
        final double expectedResult = Double.POSITIVE_INFINITY;
        Assertions.assertEquals(expectedResult, actualResult);
    }

    @Test
    void testComputeShortestDurationToPastDatesWithOnePastDate() {
        // GIVEN
        final AbsoluteDate date = AbsoluteDate.ARBITRARY_EPOCH;
        final double expectedResult = 200;
        final AbsoluteDate shiftedDate = date.shiftedBy(-expectedResult);
        // WHEN
        final double actualResult = DateDetectionAdaptableIntervalFactory.computeShortestDurationFromPastDates(date,
                shiftedDate);
        // THEN
        Assertions.assertEquals(expectedResult, actualResult);
    }

    @Test
    void testGetForwardDateDetectionInterval() {
        // GIVEN
        final Orbit initialOrbit = createOrbit();
        final double duration = 1e12;
        final AbsoluteDate targetDate = initialOrbit.getDate().shiftedBy(duration);
        final DateDetector dateDetector = new DateDetector(targetDate);
        final AdaptableInterval adaptableInterval = DateDetectionAdaptableIntervalFactory.getForwardDateDetectionInterval(
                dateDetector.getDates().stream().map(TimeStamped::getDate).toArray(AbsoluteDate[]::new));
        final AdaptableIntervalWithCounter adaptableIntervalWithCounter = new AdaptableIntervalWithCounter(adaptableInterval);
        final Propagator propagator = createPropagatorWithDateDetector(initialOrbit, dateDetector,
                adaptableIntervalWithCounter);
        // WHEN
        final SpacecraftState propagatedState = propagator.propagate(targetDate.getDate().shiftedBy(1));
        // THEN
        Assertions.assertEquals(targetDate, propagatedState.getDate());
        final int unexpectedCount = countWithDefaultMaxCheck(initialOrbit, targetDate);
        Assertions.assertTrue(unexpectedCount > adaptableIntervalWithCounter.count);
    }

    @Test
    void testGetBackwardDateDetectionInterval() {
        // GIVEN
        final Orbit initialOrbit = createOrbit();
        final double duration = -1e12;
        final AbsoluteDate targetDate = initialOrbit.getDate().shiftedBy(duration);
        final DateDetector dateDetector = new DateDetector(targetDate);
        final AdaptableInterval adaptableInterval = DateDetectionAdaptableIntervalFactory.getBackwardDateDetectionInterval(
                dateDetector.getDates().stream().map(TimeStamped::getDate).toArray(AbsoluteDate[]::new));
        final AdaptableIntervalWithCounter adaptableIntervalWithCounter = new AdaptableIntervalWithCounter(adaptableInterval);
        final Propagator propagator = createPropagatorWithDateDetector(initialOrbit, dateDetector,
                adaptableIntervalWithCounter);
        // WHEN
        final SpacecraftState propagatedState = propagator.propagate(targetDate.getDate().shiftedBy(-1));
        // THEN
        Assertions.assertEquals(targetDate, propagatedState.getDate());
        final int unexpectedCount = countWithDefaultMaxCheck(initialOrbit, targetDate);
        Assertions.assertTrue(unexpectedCount > adaptableIntervalWithCounter.count);
    }

    private int countWithDefaultMaxCheck(final Orbit initialOrbit, final AbsoluteDate targetDate) {
        final DateDetector dateDetector = new DateDetector(targetDate);
        final AdaptableInterval adaptableIntervalConstant = state -> DateDetector.DEFAULT_MAX_CHECK;
        final AdaptableIntervalWithCounter adaptableIntervalWithCounter = new AdaptableIntervalWithCounter(adaptableIntervalConstant);
        final Propagator propagator = createPropagatorWithDateDetector(initialOrbit, dateDetector,
                adaptableIntervalWithCounter);
        final double extraTime = (targetDate.isAfter(initialOrbit)) ? 1 : -1;
        propagator.propagate(targetDate.getDate().shiftedBy(extraTime));
        return adaptableIntervalWithCounter.count;
    }

    private Orbit createOrbit() {
        return new EquinoctialOrbit(1e7, 0.0001, 0., 0., 0.1, 0., PositionAngleType.MEAN,
                FramesFactory.getGCRF(), AbsoluteDate.ARBITRARY_EPOCH, Constants.EGM96_EARTH_MU);
    }

    private Propagator createPropagatorWithDateDetector(final Orbit initialOrbit, final DateDetector dateDetector,
                                                        final AdaptableIntervalWithCounter adaptableInterval) {
        return createPropagatorWithDetector(initialOrbit,
                dateDetector.withMaxCheck(adaptableInterval).withHandler(new StopOnEvent()));
    }

    private Propagator createPropagatorWithDetector(final Orbit initialOrbit,
                                                    final EventDetector eventDetector) {
        final KeplerianPropagator propagator = new KeplerianPropagator(initialOrbit);
        propagator.addEventDetector(eventDetector);
        return propagator;
    }

    private static class AdaptableIntervalWithCounter implements AdaptableInterval {

        private final AdaptableInterval interval;
        int count = 0;

        AdaptableIntervalWithCounter(final AdaptableInterval adaptableInterval) {
            this.interval = adaptableInterval;
        }

        @Override
        public double currentInterval(SpacecraftState state) {
            count++;
            return interval.currentInterval(state);
        }
    }

}
