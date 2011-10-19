/* Copyright 2011 Eumetsat. */
package eu.eumetsat.skat.utils;

import java.text.MessageFormat;
import java.util.Locale;

import org.apache.commons.math.exception.util.ExceptionContext;
import org.apache.commons.math.exception.util.ExceptionContextProvider;
import org.apache.commons.math.exception.util.Localizable;


/** This class is the base class for all specific exceptions thrown by
 * the Skat classes.

 * <p>When the skat classes throw exceptions that are specific to
 * the package, these exceptions are always subclasses of
 * SkatException. When exceptions that are already covered by the
 * standard java API should be thrown, like
 * ArrayIndexOutOfBoundsException or InvalidParameterException, these
 * standard exceptions are thrown rather than the commons-math specific
 * ones.</p>
 * <p>This class also provides utility methods to throw some standard
 * java exceptions with localized messages.</p>
 * <p>This class is almost a copy of OrekitException from the Orekit
 * library, which is published under the terms of the Apache Software
 * License V2.</p>
 *
 * @author Luc Maisonobe
 */

public class SkatException extends Exception {

    /** Serializable UID. */
    private static final long serialVersionUID = -3445247121742720233L;

    /** Exception context (may be null). */
    private final ExceptionContext context;

    /** Format specifier (to be translated). */
    private final Localizable specifier;

    /** Parts to insert in the format (no translation). */
    private final Object[] parts;

    /** Simple constructor.
     * Build an exception with a translated and formatted message
     * @param specifier format specifier (to be translated)
     * @param parts parts to insert in the format (no translation)
     */
    public SkatException(final Localizable specifier, final Object ... parts) {
        this.context   = null;
        this.specifier = specifier;
        this.parts     = (parts == null) ? new Object[0] : parts.clone();
    }

    /** Copy constructor.
     * @param exception exception to copy from
     * @since 5.1
     */
    public SkatException(final SkatException exception) {
        super(exception);
        this.context   = exception.context;
        this.specifier = exception.specifier;
        this.parts     = exception.parts.clone();
    }

    /** Simple constructor.
     * Build an exception from a cause and with a specified message
     * @param message descriptive message
     * @param cause underlying cause
     */
    public SkatException(final Localizable message, final Throwable cause) {
        super(cause);
        this.context   = null;
        this.specifier = message;
        this.parts     = new Object[0];
    }

    /** Simple constructor.
     * Build an exception from a cause and with a translated and formatted message
     * @param cause underlying cause
     * @param specifier format specifier (to be translated)
     * @param parts parts to insert in the format (no translation)
     */
    public SkatException(final Throwable cause, final Localizable specifier,
                           final Object ... parts) {
        super(cause);
        this.context   = null;
        this.specifier = specifier;
        this.parts     = (parts == null) ? new Object[0] : parts.clone();
    }

    /** Simple constructor.
     * Build an exception from an Apache Commons Math exception context context
     * @param context underlying exception context context
     * @since 6.0
     */
    public SkatException(final ExceptionContextProvider provider) {
        super(provider.getContext().getThrowable());
        this.context   = provider.getContext();
        this.specifier = null;
        this.parts     = new Object[0];
    }

    /** Gets the message in a specified locale.
     * @param locale Locale in which the message should be translated
     * @return localized message
     * @since 5.0
     */
    public String getMessage(final Locale locale) {
        return (context != null) ?
                context.getMessage(locale) :
                buildMessage(locale, specifier, parts);
    }

    /** {@inheritDoc} */
    @Override
    public String getMessage() {
        return getMessage(Locale.US);
    }

    /** {@inheritDoc} */
    @Override
    public String getLocalizedMessage() {
        return getMessage(Locale.getDefault());
    }

    /** Get the localizable specifier of the error message.
     * @return localizable specifier of the error message
     * @since 5.1
     */
    public Localizable getSpecifier() {
        return specifier;
    }

    /** Get the variable parts of the error message.
     * @return a copy of the variable parts of the error message
     * @since 5.1
     */
    public Object[] getParts() {
        return parts.clone();
    }

    /**
     * Builds a message string by from a pattern and its arguments.
     * @param locale Locale in which the message should be translated
     * @param specifier format specifier (to be translated)
     * @param parts parts to insert in the format (no translation)
     * @return a message string
     */
    private static String buildMessage(final Locale locale, final Localizable specifier,
                                       final Object ... parts) {
        return (specifier == null) ? "" : new MessageFormat(specifier.getLocalizedString(locale), locale).format(parts);
    }

    /** Create an {@link java.lang.RuntimeException} for an internal error.
     * @param cause underlying cause
     * @return an {@link java.lang.RuntimeException} for an internal error
     */
    public static RuntimeException createInternalError(final Throwable cause) {

        /** Format specifier (to be translated). */
        final Localizable specifier = SkatMessages.INTERNAL_ERROR;

        /** Parts to insert in the format (no translation). */
        // TODO: put an appropriate mail address
        final String parts     = "<TBD: put a mail adress for Skat maintenance>";

        return new RuntimeException() {

            /** Serializable UID. */
           private static final long serialVersionUID = -8979627107643864302L;

            /** {@inheritDoc} */
            @Override
            public String getMessage() {
                return buildMessage(Locale.US, specifier, parts);
            }

            /** {@inheritDoc} */
            @Override
            public String getLocalizedMessage() {
                return buildMessage(Locale.getDefault(), specifier, parts);
            }

        };

    }

    /** Create an {@link java.lang.IllegalArgumentException} for an internal error.
     * @param specifier format specifier (to be translated)
     * @param parts parts to insert in the format (no translation)
     * @return an {@link java.lang.IllegalArgumentException}
     */
    public static IllegalArgumentException createIllegalArgumentException(final Localizable specifier,
                                                                          final Object ... parts) {

        return new IllegalArgumentException() {

            /** Serializable UID. */
            private static final long serialVersionUID = 4573516034997263356L;

            /** {@inheritDoc} */
            @Override
            public String getMessage() {
                return buildMessage(Locale.US, specifier, parts);
            }

            /** {@inheritDoc} */
            @Override
            public String getLocalizedMessage() {
                return buildMessage(Locale.getDefault(), specifier, parts);
            }

        };

    }

    /** Create an {@link java.lang.IllegalStateException} for an internal error.
     * @param specifier format specifier (to be translated)
     * @param parts parts to insert in the format (no translation)
     * @return an {@link java.lang.IllegalStateException}
     */
    public static IllegalStateException createIllegalStateException(final Localizable specifier,
                                                                    final Object ... parts) {

        return new IllegalStateException() {

            /** Serializable UID. */
            private static final long serialVersionUID = 4573516034997263356L;

            /** {@inheritDoc} */
            @Override
            public String getMessage() {
                return buildMessage(Locale.US, specifier, parts);
            }

            /** {@inheritDoc} */
            @Override
            public String getLocalizedMessage() {
                return buildMessage(Locale.getDefault(), specifier, parts);
            }

        };

    }

    /** Create an {@link java.lang.NoSuchFieldException} for an internal error.
     * @param specifier format specifier (to be translated)
     * @param parts parts to insert in the format (no translation)
     * @return a {@link java.lang.NoSuchFieldException}
     */
    public static NoSuchFieldException createNoSuchFieldException(final Localizable specifier,
                                                                    final Object ... parts) {

        return new NoSuchFieldException() {

            /** Serializable UID. */
            private static final long serialVersionUID = 7993072272125446238L;

            /** {@inheritDoc} */
            @Override
            public String getMessage() {
                return buildMessage(Locale.US, specifier, parts);
            }

            /** {@inheritDoc} */
            @Override
            public String getLocalizedMessage() {
                return buildMessage(Locale.getDefault(), specifier, parts);
            }

        };

    }

}
