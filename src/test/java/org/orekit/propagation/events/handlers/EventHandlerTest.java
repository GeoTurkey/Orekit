/* Copyright 2002-2022 CS GROUP
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
package org.orekit.propagation.events.handlers;

import org.hipparchus.ode.events.Action;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.time.AbsoluteDate;

public class EventHandlerTest {

    @Test
    public void testEnums() {
        // this test is here only for test coverage ...

        Assert.assertEquals(5, Action.values().length);
        Assert.assertSame(Action.STOP,              Action.valueOf("STOP"));
        Assert.assertSame(Action.RESET_STATE,       Action.valueOf("RESET_STATE"));
        Assert.assertSame(Action.RESET_DERIVATIVES, Action.valueOf("RESET_DERIVATIVES"));
        Assert.assertSame(Action.RESET_EVENTS,      Action.valueOf("RESET_EVENTS"));
        Assert.assertSame(Action.CONTINUE,          Action.valueOf("CONTINUE"));

    }

    @Test
    public void testIssue721() {

        // Create detector
        final Detector detector = new Detector();
        Assert.assertFalse(detector.isInitialized());

        // Create handler
        final Handler handler = new Handler();
        handler.init(null, null, detector);
        Assert.assertTrue(detector.isInitialized());

    }

    private static class Detector implements EventDetector {

        private boolean initialized;

        public Detector() {
            this.initialized = false;
        }

        public boolean isInitialized() {
            return initialized;
        }

        @Override
        public void init(SpacecraftState s0, AbsoluteDate t) {
            initialized = true;
        }

        @Override
        public double g(SpacecraftState s) {
            return 0;
        }

        @Override
        public double getThreshold() {
            return 0;
        }

        @Override
        public double getMaxCheckInterval() {
            return 0;
        }

        @Override
        public int getMaxIterationCount() {
            return 0;
        }

        @Override
        public Action eventOccurred(SpacecraftState s, boolean increasing) {
            return Action.CONTINUE;
        }
    }

    private static class Handler implements EventHandler<Detector> {

        @Override
        public void init(SpacecraftState initialState, AbsoluteDate target,
                         Detector detector) {
            detector.init(initialState, target);
        }

        @Override
        public Action eventOccurred(SpacecraftState s, Detector detector,
                                    boolean increasing) {
            return detector.eventOccurred(s, increasing);
        }

    }
}

