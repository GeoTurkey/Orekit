/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.io.OutputStream;
import java.io.PrintStream;
import java.text.Format;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.util.TimeZone;

import org.apache.commons.math.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;

/** MonitorMono for time-dependent values writing their evolution in a csv or tsv file.
 * <p>
 * CSV stands for Comma Separated Values and TSV stands for Tab Separated Values.
 * Despite its name, this class support any string as a separator, as long as there
 * is no ambiguity with respect to the fields themselves. This means users should
 * not use "." as the separator if the decimal format they specify uses this character
 * as a decimal separator for floating values. No verification is done about this.
 * </p>
 * <p>
 * The file created has an optional header containing the file creation date, the
 * reference date used to compute the first column offset, and a description of all
 * columns. After the optional header, the file content is composed of lines, each
 * line having a fixed number of fields written using the user-provided format and
 * separator. The first column is always a date offset in days for the current line
 * with respect to the reference date.
 * </p>
 * <p>
 * As values are changed and this monitor is notified of such changes, the values
 * are dispatched in their assigned columns but the line is not written yet. When
 * the date of an updated value is farther than a specified threshold from the
 * current date, the line is considered completed and written, and a new line is
 * started. This implies that a line is displayed when the first value for the next
 * line is updated, so there is a slight delay. If a value has not been updated
 * while a line is completed, the last value which was updated several lines before
 * is written again. So the number of fields is guaranteed to be constant, and the
 * file can contain some values that are updated often (like spacecraft longitude
 * for example) and some values that are updated less frequently (like spacecraft
 * mass for example). The last line is written when monitoring stops.
 * </p>
 * 
 * @author Luc Maisonobe
 */
public class CsvFileMonitor implements MonitorMono, MonitorDuo {

    /** Index of the first (or unique) monitored spacecraft. */
    private final int index1;

    /** Index of the second monitored spacecraft. */
    private final int index2;

    /** Output file. */
    private final PrintStream out;

    /** Header lines marker. */
    private final String headerMarker;

    /** Fields separator. */
    private final String separator;

    /** Format to use for writing fields. */
    private final Format format;

    /** Reference date to compute first column. */
    private final AbsoluteDate referenceDate;

    /** Tolerance in second below which values are gathered in the same line. */
    private final double dateTolerance;

    /** Total number of columns (not counting the first date offset column). */
    private int columns;

    /** First column for each monitored value. */
    private final Map<String, Integer> firstColumns;

    /** Date of the line under construction. */
    private AbsoluteDate currentDate;

    /** Current fields. */
    private String[] currentFields;

    /** Simple constructor.
     * @param index index of the unique monitored spacecraft
     * @param outputStream output stream
     * @param headerMarker marker for starting header lines, if null no header
     * will be generated (a typical marker is "# ")
     * @param separator fields separator (typical separators are "," or "\t")
     * @param format format to use for writing fields
     * @param referenceDate reference date to compute offset in first column
     * @param dateTolerance tolerance in seconds below which values are gathered
     * in the same line
     * @exception OrekitException if UTC time scale cannot be retrieved
     */
    public CsvFileMonitor(final int index,
                          final OutputStream outputStream, final String headerMarker,
                          final String separator, final Format format,
                          final AbsoluteDate referenceDate, final double dateTolerance)
        throws OrekitException {
        this(index, -1, outputStream, headerMarker, separator, format, referenceDate, dateTolerance);
    }

    /** Simple constructor.
     * @param index1 index of the first monitored spacecraft
     * @param index2 index of the second monitored spacecraft
     * @param outputStream output stream
     * @param headerMarker marker for starting header lines, if null no header
     * will be generated (a typical marker is "# ")
     * @param separator fields separator (typical separators are "," or "\t")
     * @param format format to use for writing fields
     * @param referenceDate reference date to compute offset in first column
     * @param dateTolerance tolerance in seconds below which values are gathered
     * in the same line
     * @exception OrekitException if UTC time scale cannot be retrieved
     */
    public CsvFileMonitor(final int index1, final int index2,
                          final OutputStream outputStream, final String headerMarker,
                          final String separator, final Format format,
                          final AbsoluteDate referenceDate, final double dateTolerance)
        throws OrekitException {

        this.index1        = index1;
        this.index2        = index2;
        this.out           = new PrintStream(outputStream);
        this.headerMarker  = headerMarker;
        this.separator     = separator;
        this.format        = format;
        this.referenceDate = referenceDate;
        this.dateTolerance = dateTolerance;
        this.firstColumns  = new HashMap<String, Integer>();
        this.columns       = 0;

        // at construction, we don't have yet a current date,
        // this will be used as a flag to check if we can accept
        // calls to startMonitoring
        this.currentDate   = null;
        this.currentFields = null;

        if (headerMarker != null) {

            final TimeScale utc = TimeScalesFactory.getUTC();
            final Date now = Calendar.getInstance(TimeZone.getTimeZone("Etc/UTC")).getTime();

            // start the header
            out.println(headerMarker + " file generated on " + new AbsoluteDate(now, utc).toString(utc));
            out.println(headerMarker);
            out.println(headerMarker + " column 1: date offset since " + referenceDate.toString(utc));

        }

    }

    /** {@inheritDoc} */
    public void startMonitoring(MonitorableMono monitorable)
        throws IllegalArgumentException, IllegalStateException {
        startMonitoring(monitorable.getValue(index1).length, monitorable.getName());
    }

    /** {@inheritDoc} */
    public void startMonitoring(MonitorableDuo monitorable)
        throws IllegalArgumentException, IllegalStateException {
        startMonitoring(monitorable.getValue(index1, index2).length, monitorable.getName());
    }

    /** Notifies a monitor that monitoring should start for the specified value.
     * @param dimension dimension of the monitored parameter
     * @param name name of the monitored parameter
     * @exception IllegalArgumentException if a {@link MonitorableMono} with the same name
     * is already monitored
     * @exception IllegalStateException if {@link #valueChanged(MonitorableMono)
     * valueChanged} has already been called at least once.
     */
    private void startMonitoring(final int dimension, final String name) {

        // check if we have started monitoring or not
        if (currentDate != null) {
            throw SkatException.createIllegalStateException(SkatMessages.MONITORING_ALREADY_STARTED, name);
        }

        // check if a similar  value is already monitored
        if (firstColumns.containsKey(name)) {
            throw SkatException.createIllegalArgumentException(SkatMessages.VALUE_ALREADY_MONITORED, name);
        }

        // write header
        if (headerMarker != null) {
            if (dimension == 1) {
                out.println(headerMarker + " column " + (columns + 2) + ": " + name);
            } else {
                for (int i = 0; i < dimension; ++i) {
                    out.println(headerMarker + " column " + (columns + i + 2) + ": " + name + "[" + i + "]");
                }
            }
        }

        // assign columns to the monitorable
        firstColumns.put(name, columns);
        columns += dimension;

    }

    /** {@inheritDoc} */
    public void valueChanged(MonitorableMono monitorable) {

        final String name = monitorable.getName();

        // check if the monitorable is monitored
        Integer first = firstColumns.get(name);
        if (first == null) {
            throw SkatException.createIllegalArgumentException(SkatMessages.VALUE_NOT_MONITORED,
                                                               name);
        }

        // we know the monitorable is managed, we can extract its data
        valueChanged(first, monitorable.getDate(), monitorable.getValue(index1));

    }

    /** {@inheritDoc} */
    public void valueChanged(MonitorableDuo monitorable) {

        final String name = monitorable.getName();

        // check if the monitorable is monitored
        Integer first = firstColumns.get(name);
        if (first == null) {
            throw SkatException.createIllegalArgumentException(SkatMessages.VALUE_NOT_MONITORED,
                                                               name);
        }

        // we know the monitorable is managed, we can extract its data
        valueChanged(first, monitorable.getDate(), monitorable.getValue(index1, index2));

    }

    /** Monitor a time-dependent value.
     * <p>
     * This method is called each time a monitored value changes.
     * </p>
     * @param first first column of the monitorable
     * @param date current date
     * @param value time-dependent value
     */
    private void valueChanged(final int first, final AbsoluteDate date, final double[] value) {

        if (currentDate == null) {
            // this is the first time this method is called,
            // we initialize the fields array with dummy values
            currentFields = new String[columns];
            Arrays.fill(currentFields, format.format(Double.NaN));
            currentDate = date;
        } else if (FastMath.abs(date.durationFrom(currentDate)) > dateTolerance) {
            // the change date is far from the previous line,
            // we consider the previous line is completed, we can write it to the file
            writeCompletedLine();
            currentDate = date;
        }

        // update the fields assigned to the monitorable
        for (int i = 0; i < value.length; ++i) {
            currentFields[first + i] = format.format(value[i]); 
        }
        
    }

    /** {@inheritDoc} */
    public void stopMonitoring() {
        if (currentDate != null) {
            writeCompletedLine();
        }
        out.close();
    }

    /** Write the current line. */
    private void writeCompletedLine() {

        // write the date offset column
        out.print(format.format(currentDate.durationFrom(referenceDate) / Constants.JULIAN_DAY));

        // write the values fields
        for (final String field : currentFields) {
            out.print(separator);
            out.print(field);
        }

        // complete the line
        out.println();

    }

}
