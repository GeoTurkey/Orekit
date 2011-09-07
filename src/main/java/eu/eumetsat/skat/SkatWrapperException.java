/* Copyright 2011 Eumetsat. */
package eu.eumetsat.skat;

/** This class is a wrapper for checked exceptions.
 * <p>
 * The purpose ot his class is to wrap {@link SkatException}
 * instances so they can be conveyed through Apache Commons Math level,
 * for example during optimization.
 * </p>
 *
 * @author Luc Maisonobe
 */

public class SkatWrapperException extends RuntimeException {

    /** Serializable UID. */
    private static final long serialVersionUID = 2341589396082152362L;

    /** Wrapped exception. */
    private final SkatException wrapped;

    /** Build an exception wrapping a {@link SkatException}.
     * @param SkatException SkatException to wrap
     */
    public SkatWrapperException(final SkatException wrapped) {
        this.wrapped = wrapped;
    }

    /** Get the wrapped {@link SkatException}
     * @return wrapped {@link SkatException}
     */
    public SkatException getWrappedException() {
        return wrapped;
    }

}
