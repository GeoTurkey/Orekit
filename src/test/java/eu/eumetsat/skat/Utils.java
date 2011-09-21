/* Copyright 2002-2011 CS Communication & Systèmes
 * Licensed to CS Communication & Systèmes (CS) under one or more
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
package eu.eumetsat.skat;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.net.URISyntaxException;
import java.util.Map;

import org.junit.Assert;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.JPLEphemeridesLoader;
import org.orekit.data.DataProvidersManager;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.frames.FramesFactory;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;

/** Simple parser for key/value files.
 * <p>
 * This file is a slightly modified copy (changed package name, removed constants)
 * of the similar file present in the Orekit library tests,
 * which is distributed under the terms of the Apache Software
 * License V2.
 * </p>
 */
public class Utils {

    public static void setDataRoot(String root) {
        try {
            Utils.clearFactory(CelestialBodyFactory.class);
            CelestialBodyFactory.clearCelestialBodyLoaders();
            Utils.clearFactory(FramesFactory.class);
            FramesFactory.clearEOP1980HistoryLoaders();
            FramesFactory.clearEOP2000HistoryLoaders();
            Utils.clearFactory(TimeScalesFactory.class, TimeScale.class);
            TimeScalesFactory.clearUTCTAILoaders();
            Utils.clearJPLEphemeridesConstants();
            GravityFieldFactory.clearPotentialCoefficientsReaders();
            DataProvidersManager.getInstance().clearProviders();
            DataProvidersManager.getInstance().clearLoadedDataNames();
            StringBuffer buffer = new StringBuffer();
            for (String component : root.split(":")) {
                String componentPath;
                componentPath = Utils.class.getClassLoader().getResource(component).toURI().getPath();
                if (buffer.length() > 0) {
                    buffer.append(System.getProperty("path.separator"));
                }
                buffer.append(componentPath);
            }
            System.setProperty(DataProvidersManager.OREKIT_DATA_PATH, buffer.toString());
        } catch (URISyntaxException e) {
            throw new RuntimeException(e);
        }
    }

    private static void clearFactory(Class<?> factoryClass) {
        try {
            for (Field field : factoryClass.getDeclaredFields()) {
                if (Modifier.isStatic(field.getModifiers()) &&
                    Map.class.isAssignableFrom(field.getType())) {
                    field.setAccessible(true);
                    ((Map<?, ?>) field.get(null)).clear();
                }
            }
        } catch (IllegalAccessException iae) {
            Assert.fail(iae.getMessage());
        }
    }

    private static void clearFactory(Class<?> factoryClass, Class<?> cachedFieldsClass) {
        try {
            for (Field field : factoryClass.getDeclaredFields()) {
                if (Modifier.isStatic(field.getModifiers()) &&
                        cachedFieldsClass.isAssignableFrom(field.getType())) {
                    field.setAccessible(true);
                    field.set(null, null);
                }
            }
        } catch (IllegalAccessException iae) {
            Assert.fail(iae.getMessage());
        }
    }

    @SuppressWarnings("unchecked")
    private static void clearJPLEphemeridesConstants() {
        try {
            for (Field field : JPLEphemeridesLoader.class.getDeclaredFields()) {
                if (field.getName().equals("CONSTANTS")) {
                    field.setAccessible(true);
                    ((Map<String, Double>) field.get(null)).clear();
                }
            }
        } catch (IllegalAccessException iae) {
            Assert.fail(iae.getMessage());
        }
    }


}
