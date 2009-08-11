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
package org.orekit.utils;

import java.io.Serializable;
import java.util.Date;

import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.time.TimeStamped;


/** Container class for Earth Orientation Parameters provided by IERS.
 * <p>Instances of this class correspond to lines from either the
 * EOP C 04 yearly files or the bulletin B monthly files.</p>
 * @author Luc Maisonobe
 * @see org.orekit.frames.Frame
 * @version $Revision:1665 $ $Date:2008-06-11 12:12:59 +0200 (mer., 11 juin 2008) $
 */
public class TimeStampedEntry implements TimeStamped, Serializable {

    /** Serializable UID. */
    private static final long serialVersionUID = -6655551509699738718L;

    /** Entry date (modified julian day, 00h00 UTC scale). */
    private final int mjd;

    /** Various entry fields. */
    private final double[] fields;

    /** Entry date (absolute date). */
    private final AbsoluteDate date;

    /** Simple constructor.
     * @param mjd entry date
     * @param fields various entry fields
     * @exception OrekitException if the UTC scale cannot be initialized
     */
    public TimeStampedEntry(final int mjd, final double... fields)
        throws OrekitException {

        this.mjd    = mjd;
        this.fields = fields.clone();

        // convert mjd date at 00h00 UTC to absolute date
        final long javaTime = (mjd - 40587) * 86400000l;
        date = new AbsoluteDate(new Date(javaTime), TimeScalesFactory.getUTC());

    }

    /** Get the entry date (modified julian day, 00h00 UTC scale).
     * @return entry date
     * @see #getDate()
     */
    public int getMjd() {
        return mjd;
    }

    /** Get the entry date (absolute date).
     * @return entry date
     * @see #getMjd()
     */
    public AbsoluteDate getDate() {
        return date;
    }

    /** Get some field among the entry.
     * @param id index of the field to get
     * @return the desired field
     */
    public double getField(final int id) {
        return fields[id];
    }

}
