/* Copyright 2011 Eumetsat.
 */
package eu.eumetsat.skat.utils;

import java.util.Locale;
import java.util.MissingResourceException;
import java.util.ResourceBundle;

import org.apache.commons.math.exception.util.Localizable;

/**
 * Enumeration for localized messages formats.
 * <p>
 * The constants in this enumeration represent the available
 * formats as localized strings. These formats are intended to be
 * localized using simple properties files, using the constant
 * name as the key and the property value as the message format.
 * The source English format is provided in the constants themselves
 * to serve both as a reminder for developers to understand the parameters
 * needed by each format, as a basis for translators to create
 * localized properties files, and as a default format if some
 * translation is missing.
 * </p>
 * <p>
 * This class is heavily based on similar classes from Orekit and Apache
 * Commons Math which are published under the terms of the Apache
 * Software License V2.
 * </p>
 */
public enum SkatMessages implements Localizable {

    // CHECKSTYLE: stop JavadocVariable check

    INTERNAL_ERROR("internal error, contact maintenance at {0}"),
    MONITORING_ALREADY_STARTED("cannot monitor {0}, monitoring already started"),
    NOT_EARTH_FRAME("frame {0} is not an Earth frame"),
    NOT_INERTIAL_FRAME("frame {0} is not pseudo-inertial"),
    UNABLE_TO_FIND_RESOURCE("unable to find resource {0}"),
    UNKNOWN_FRAME("unknown frame {0}"),
    VALUE_ALREADY_MONITORED("value {0} is already monitored"),
    VALUE_NOT_MONITORED("value {0} is not monitored"),
    WRONG_TYPE("type error at line {0} of input file {1}, expected {2} but was {3}"),
    MISSING_INPUT_DATA("missing data near line {0} of input file {1}: {2}"),
    UNSUPPORTED_KEY("unsupported key \"{0}\" at line {1} of input file {2}, supported keys: {3}"),
    UNKNOWN_SPACECRAFT("unknown spacecraft {0}, known spacecrafts: {1}"),
    SPACECRAFT_MANAGED_TWICE("spacecraft {0} is managed by several propagation scenario components"),
    SPACECRAFT_NOT_MANAGED("spacecraft {0} is not managed by any propagation scenario components"),
    NO_ESTIMATED_STATE("no estimated state available for {0} at cycle {1}, maybe an orbit determination component is missing"),
    NO_THEORETICAL_MANEUVERS_STATE("no theoretical maneuvers available for {0} at cycle {1}, maybe a control loop component is missing for this cycle"),
    NO_PERFORMED_MANEUVERS_STATE("no performed maneuvers available for {0} at cycle {1}, maybe a maneuver realization component is missing"),
    NO_EPHEMERIS_STATE("no ephemeris available for {0} at cycle {1}, maybe a propagation component is missing"),
    NO_END_STATE("no end state available for {0} at cycle {1}, maybe a propagation component is missing");

    // CHECKSTYLE: resume JavadocVariable check

    /** Base name of the resource bundle in classpath. */
    private static final String RESOURCE_BASE_NAME = "assets/localization/SkatMessages";

    /** Source English format. */
    private final String sourceFormat;

    /** Simple constructor.
     * @param sourceFormat source English format to use when no
     * localized version is available
     */
    private SkatMessages(final String sourceFormat) {
        this.sourceFormat = sourceFormat;
    }

    /** {@inheritDoc} */
    public String getSourceString() {
        return sourceFormat;
    }

    /** {@inheritDoc} */
    public String getLocalizedString(final Locale locale) {
        try {
            final ResourceBundle bundle = ResourceBundle.getBundle(RESOURCE_BASE_NAME, locale);
            if (bundle.getLocale().getLanguage().equals(locale.getLanguage())) {
                final String translated = bundle.getString(name());
                if ((translated != null) &&
                    (translated.length() > 0) &&
                    (!translated.toLowerCase().contains("missing translation"))) {
                    // the value of the resource is the translated format
                    return translated;
                }
            }

        } catch (MissingResourceException mre) {
            // do nothing here
        }

        // either the locale is not supported or the resource is not translated or
        // it is unknown: don't translate and fall back to using the source format
        return sourceFormat;

    }

}
