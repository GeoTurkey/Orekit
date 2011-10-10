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

/** Monitor for time-dependent values writing their evolution in a csv or tsv file.
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
public class CsvFileMonitor implements Monitor {

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
    public CsvFileMonitor(final OutputStream outputStream, final String headerMarker,
                          final String separator, final Format format,
                          final AbsoluteDate referenceDate, final double dateTolerance)
        throws OrekitException {

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
            out.println(headerMarker + "file generated on " + new AbsoluteDate(now, utc).toString(utc));
            out.println(headerMarker);
            out.println(headerMarker + "column 1: date offset since " + referenceDate.toString(utc));

        }

    }

    /** {@inheritDoc} */
    public void startMonitoring(Monitorable monitorable)
        throws IllegalArgumentException, IllegalStateException {

        // retrieve monitorable meta-data
        final String name = monitorable.getName();
        final int n       = monitorable.getValue().length;

        // check if we have started monitoring or not
        if (currentDate != null) {
            throw SkatException.createIllegalStateException(SkatMessages.MONITORING_ALREADY_STARTED, name);
        }

        // check if a similar  value is already monitored
        if (firstColumns.containsKey(monitorable.getName())) {
            throw SkatException.createIllegalArgumentException(SkatMessages.VALUE_ALREADY_MONITORED, name);
        }

        // write header
        if (headerMarker != null) {
            if (n == 1) {
                out.println(headerMarker + "column " + (columns + 2) + ": " + name);
            } else {
                for (int i = 0; i < n; ++i) {
                    out.println(headerMarker + "column " + (columns + i + 2) + ": " + name + "[" + i + "]");
                }
            }
        }

        // assign columns to the monitorable
        firstColumns.put(name, columns);
        columns += n;

    }

    /** {@inheritDoc} */
    public void valueChanged(Monitorable monitorable) {

        // check if the monitorable is monitored
        Integer first = firstColumns.get(monitorable.getName());
        if (first == null) {
            throw SkatException.createIllegalArgumentException(SkatMessages.VALUE_NOT_MONITORED,
                                                               monitorable.getName());
        }

        if (currentDate == null) {
            // this is the first time this method is called,
            // we initialize the fields array with dummy values
            currentFields = new String[columns];
            Arrays.fill(currentFields, format.format(Double.NaN));
            currentDate = monitorable.getDate();
        } else if (FastMath.abs(monitorable.getDate().durationFrom(currentDate)) > dateTolerance) {
            // the change date is far from the previous line,
            // we consider the previous line is completed, we can write it to the file
            writeCompletedLine();
            currentDate = monitorable.getDate();
        }

        // update the fields assigned to the monitorable
        final double[] value = monitorable.getValue();
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
