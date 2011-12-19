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
    REFERENCE_MANEUVER_NOT_FOUND("reference maneuver {0} not be found (line {1} of input file {2})"),
    MANEUVER_MAY_OCCUR_BEFORE_CYCLE("maneuver {0} may occur up to {1} seconds before cycle start (line {2} of input file {3})"),
    MANEUVER_MAY_OCCUR_AFTER_CYCLE("maneuver {0} may occur up to {1} seconds after cycle end (line {2} of input file {3})"),
    UNSUPPORTED_KEY("unsupported key \"{0}\" at line {1} of input file {2}, supported keys: {3}"),
    UNKNOWN_SPACECRAFT("unknown spacecraft {0}, known spacecrafts: {1}"),
    SPACECRAFT_MANAGED_TWICE("spacecraft {0} is managed by several propagation scenario components"),
    SPACECRAFT_NOT_MANAGED("spacecraft {0} is not managed by any propagation scenario components"),
    NO_ESTIMATED_STATE("no estimated state available for {0} at cycle {1}, maybe an orbit determination component is missing"),
    NO_MANEUVERS_STATE("no maneuvers available for {0} at cycle {1}, maybe a control loop component is missing for this cycle"),
    NO_EPHEMERIS_STATE("no ephemeris available for {0} at cycle {1}, maybe a propagation component is missing"),
    NO_END_STATE("no end state available for {0} at cycle {1}, maybe a propagation component is missing"),
    WRONG_SPLIT_DT("split maneuver min dt: {0}, must be greater than 0.0"),
    WRONG_MISS_THRESHOLD("invalid maneuver miss threshold: {0}, must be between 0.0 and 1.0"),
    WRONG_COUPLING("invalid coupling ratio: {0}, must be between -1.0 and 1.0"),
    ALIGNED_COUPLING_AXES("coupling axis ({0}, {1}, {2}) is aligned with the nominal thrust direction"),
    PHASING_NUMBERS_NOT_MUTUALLY_PRIMES("number of orbits per cycle ({0}) and days per cycle ({1}) are not mutually primes"),
    NO_ECLIPSE_AROUND_DATE("no eclipse around date {0}"),
    INITIAL_MASS_LARGER_THAN_BOL_MASS("initial mass ({0} kg) larger than Begin Of Life mass ({1} kg)"),
    NON_INCREASING_MASSES_IN_THRUST_CALIBRATION_CURVE("non increasing masses in thrust decay law: {0} > {1}"),
    NON_INCREASING_MASSES_IN_ISP_CALIBRATION_CURVE("non increasing masses in ISP decay law: {0} > {1}"),
    UNSORTED_LONGITUDES("unsorted longitude limits: {0} >= {1}"),
    NO_CONVERGENCE("no convergence after {0} iterations on control law(s): {1}");

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
