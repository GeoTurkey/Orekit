/* Copyright 2002-2014 CS Systèmes d'Information
 * Licensed to CS Systèmes d'Information (CS) under one or more
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
package org.orekit.errors;

import org.apache.commons.math3.exception.util.ExceptionContextProvider;
import org.apache.commons.math3.exception.util.Localizable;

/** This class is the base class for all specific exceptions thrown by
 * during the propagation computation.
 *
 * @author Luc Maisonobe
 */
public class PropagationException extends OrekitException {

    /** Serializable UID. */
    private static final long serialVersionUID = 1684307015196169376L;

    /** Simple constructor.
     * Build an exception with a translated and formatted message
     * @param specifier format specifier (to be translated)
     * @param parts parts to insert in the format (no translation)
     */
    public PropagationException(final Localizable specifier, final Object ... parts) {
        super(specifier, parts);
    }

    /** Simple constructor.
     * Build an exception from a cause and with a specified message
     * @param cause underlying cause
     * @param specifier format specifier (to be translated)
     * @param parts parts to insert in the format (no translation)
     */
    public PropagationException(final Throwable cause, final Localizable specifier,
                                final Object ... parts) {
        super(cause, specifier, parts);
    }

    /** Simple constructor.
     * Build an exception wrapping an {@link OrekitException} instance
     * @param exception underlying cause
     */
    public PropagationException(final OrekitException exception) {
        super(exception);
    }

    /** Simple constructor.
     * Build an exception wrapping an Apache Commons Math exception context exception
     * @param provider underlying cause
     */
    public PropagationException(final ExceptionContextProvider provider) {
        super(provider);
    }

    /** Recover a PropagationException, possibly embedded in a {@link OrekitException}.
     * <p>
     * If the {@code OrekitException} does not embed a PropagationException, a
     * new one will be created.
     * </p>
     * @param oe OrekitException to analyze
     * @return a (possibly embedded) PropagationException
     */
    public static PropagationException unwrap(final OrekitException oe) {

        for (Throwable t = oe; t != null; t = t.getCause()) {
            if (t instanceof PropagationException) {
                return (PropagationException) t;
            }
        }

        return new PropagationException(oe);

    }

    /** Recover a PropagationException, possibly embedded in an {@link ExceptionContextProvider}.
     * <p>
     * If the {@code ExceptionContextProvider} does not embed a PropagationException, a
     * new one will be created.
     * </p>
     * @param provider ExceptionContextProvider to analyze
     * @return a (possibly embedded) PropagationException
     */
    public static PropagationException unwrap(final ExceptionContextProvider provider) {

        for (Throwable t = provider.getContext().getThrowable(); t != null; t = t.getCause()) {
            if (t instanceof OrekitException) {
                if (t instanceof PropagationException) {
                    return (PropagationException) t;
                } else {
                    return new PropagationException((OrekitException) t);
                }
            }
        }

        return new PropagationException(provider);

    }

}