/* Copyright 2002-2008 CS Communication & Systèmes
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
package org.orekit.time;

/** International Atomic Time.
 * <p>This is intended to be accessed thanks to the {@link TimeScalesFactory} class,
 * so there is no public constructor.</p>
 * @author Luc Maisonobe
 * @see AbsoluteDate
 * @version $Revision:1665 $ $Date:2008-06-11 12:12:59 +0200 (mer., 11 juin 2008) $
 */
public class TAIScale implements TimeScale {

    /** Serializable UID. */
    private static final long serialVersionUID = -4084040237742697106L;

    /** Package private constructor for the factory.
     */
    TAIScale() {
    }

    /** Get the unique instance of this class.
     * @return the unique instance
     * @deprecated since 4.1 replaced by {@link TimeScalesFactory#getTAI()}
     */
    @Deprecated
    public static TAIScale getInstance() {
        return TimeScalesFactory.getTAI();
    }

    /** {@inheritDoc} */
    public double offsetFromTAI(final AbsoluteDate taiTime) {
        return 0;
    }

    /** {@inheritDoc} */
    public double offsetToTAI(final DateComponents date, final TimeComponents time) {
        return 0;
    }

    /** {@inheritDoc} */
    public String getName() {
        return "TAI";
    }

    /** {@inheritDoc} */
    public String toString() {
        return getName();
    }

}
