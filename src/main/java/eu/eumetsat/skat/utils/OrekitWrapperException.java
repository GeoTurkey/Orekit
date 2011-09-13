/* Copyright 2011 Eumetsat. */
package eu.eumetsat.skat.utils;

import org.orekit.errors.OrekitException;


/** This class is a wrapper for checked exceptions.
 * <p>
 * The purpose ot his class is to wrap {@link OrekitException}
 * instances so they can be conveyed through Apache Commons Math level,
 * for example during optimization.
 * </p>
 *
 * @author Luc Maisonobe
 */

public class OrekitWrapperException extends RuntimeException {

    /** Serializable UID. */
    private static final long serialVersionUID = -3214953166300497462L;

    /** Wrapped exception. */
    private final OrekitException wrapped;

    /** Build an exception wrapping a {@link OrekitException}.
     * @param orekitException OrekitException to wrap
     */
    public OrekitWrapperException(final OrekitException wrapped) {
        this.wrapped = wrapped;
    }

    /** Get the wrapped {@link SkatException}
     * @return wrapped {@link SkatException}
     */
    public OrekitException getWrappedException() {
        return wrapped;
    }

}
