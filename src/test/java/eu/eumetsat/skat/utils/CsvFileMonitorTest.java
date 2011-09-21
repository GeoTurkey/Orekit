/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.io.ByteArrayOutputStream;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.text.Format;
import java.util.Locale;

import junit.framework.Assert;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.orekit.Utils;
import org.orekit.errors.OrekitException;
import org.orekit.time.AbsoluteDate;


public class CsvFileMonitorTest {

    private ByteArrayOutputStream out;
    private Format format;
    private BasicMonitorable mass;
    private BasicMonitorable position;
    private BasicMonitorable velocity;

    @Test
    public void testEmpty() throws OrekitException {
        CsvFileMonitor monitor = new CsvFileMonitor(out, null, ",", format, AbsoluteDate.J2000_EPOCH, 1.0);
        monitor.stopMonitoring();
        Assert.assertEquals("", out.toString());
    }

    @Test
    public void testSmallHeader() throws OrekitException {
        CsvFileMonitor monitor = new CsvFileMonitor(out, "# ", ",", format, AbsoluteDate.J2000_EPOCH, 1.0);
        monitor.stopMonitoring();
        String[] lines = out.toString().split(System.getProperty("line.separator"));
        Assert.assertEquals(3, lines.length);
        Assert.assertTrue(lines[0].startsWith("# file generated on "));
        Assert.assertEquals("# ", lines[1]);
        Assert.assertEquals("# column 1: date offset since 2000-01-01T11:58:55.816", lines[2]);
    }

    @Test
    public void testNormalHeader() throws OrekitException {
        CsvFileMonitor monitor = new CsvFileMonitor(out, "# ", ",", format, AbsoluteDate.J2000_EPOCH, 1.0);
        mass.register(monitor);
        position.register(monitor);
        velocity.register(monitor);
        monitor.stopMonitoring();
        String[] lines = out.toString().split(System.getProperty("line.separator"));
        Assert.assertEquals(10, lines.length);
        Assert.assertTrue(lines[0].startsWith("# file generated on "));
        Assert.assertEquals("# ", lines[1]);
        Assert.assertEquals("# column 1: date offset since 2000-01-01T11:58:55.816", lines[2]);
        Assert.assertEquals("# column 2: mass",        lines[3]);
        Assert.assertEquals("# column 3: position[0]", lines[4]);
        Assert.assertEquals("# column 4: position[1]", lines[5]);
        Assert.assertEquals("# column 5: position[2]", lines[6]);
        Assert.assertEquals("# column 6: velocity[0]", lines[7]);
        Assert.assertEquals("# column 7: velocity[1]", lines[8]);
        Assert.assertEquals("# column 8: velocity[2]", lines[9]);
    }

    @Test
    public void testSmallNominalCase() throws OrekitException {
        CsvFileMonitor monitor = new CsvFileMonitor(out, "# ", ",", format, AbsoluteDate.J2000_EPOCH, 1.0);
        mass.register(monitor);
        position.register(monitor);
        velocity.register(monitor);
        AbsoluteDate t0 = AbsoluteDate.J2000_EPOCH.shiftedBy(10.0);
        mass.setDateAndValue(t0,     new double[] { 1000.0 });
        velocity.setDateAndValue(t0, new double[] { 44444.44, 55555.55, 66666.66 });
        position.setDateAndValue(t0, new double[] { 11111.11, 22222.22, 33333.33 });
        AbsoluteDate t1 = AbsoluteDate.J2000_EPOCH.shiftedBy(20.0);
        velocity.setDateAndValue(t1, new double[] { 11111.11, 22222.22, 33333.33 });
        position.setDateAndValue(t1, new double[] { 44444.44, 55555.55, 66666.66 });
        monitor.stopMonitoring();
        String[] lines = out.toString().split(System.getProperty("line.separator"));
        Assert.assertEquals(12, lines.length);
        Assert.assertEquals("10.00,1000.00,11111.11,22222.22,33333.33,44444.44,55555.55,66666.66", lines[10]);
        Assert.assertEquals("20.00,1000.00,44444.44,55555.55,66666.66,11111.11,22222.22,33333.33", lines[11]);
    }

    @Test
    public void testShortStep() throws OrekitException {
        CsvFileMonitor monitor = new CsvFileMonitor(out, "# ", ",", format, AbsoluteDate.J2000_EPOCH, 1.0);
        mass.register(monitor);
        position.register(monitor);
        velocity.register(monitor);
        AbsoluteDate t0 = AbsoluteDate.J2000_EPOCH.shiftedBy(10.0);
        mass.setDateAndValue(t0,     new double[] { 1000.0 });
        velocity.setDateAndValue(t0, new double[] { 44444.44, 55555.55, 66666.66 });
        position.setDateAndValue(t0, new double[] { 11111.11, 22222.22, 33333.33 });
        AbsoluteDate t1 = AbsoluteDate.J2000_EPOCH.shiftedBy(10.7);
        velocity.setDateAndValue(t1, new double[] { 77777.77, 88888.88, 99999.99 });
        AbsoluteDate t2 = AbsoluteDate.J2000_EPOCH.shiftedBy(20.0);
        velocity.setDateAndValue(t2, new double[] { 11111.11, 22222.22, 33333.33 });
        position.setDateAndValue(t2, new double[] { 44444.44, 55555.55, 66666.66 });
        monitor.stopMonitoring();
        String[] lines = out.toString().split(System.getProperty("line.separator"));
        Assert.assertEquals(12, lines.length);
        Assert.assertEquals("10.00,1000.00,11111.11,22222.22,33333.33,77777.77,88888.88,99999.99", lines[10]);
        Assert.assertEquals("20.00,1000.00,44444.44,55555.55,66666.66,11111.11,22222.22,33333.33", lines[11]);
    }

    @Test(expected=IllegalArgumentException.class)
    public void testDuplicatedMonitorable() throws OrekitException {
        CsvFileMonitor monitor = new CsvFileMonitor(out, "# ", ",", format, AbsoluteDate.J2000_EPOCH, 1.0);
        mass.register(monitor);
        position.register(monitor);
        velocity.register(monitor);
        new BasicMonitorable("mass", 1).register(monitor);
    }

    @Test(expected=IllegalArgumentException.class)
    public void testNotMonitored() throws OrekitException {
        CsvFileMonitor monitor = new CsvFileMonitor(out, "# ", ",", format, AbsoluteDate.J2000_EPOCH, 1.0);
        monitor.valueChanged(new BasicMonitorable("unknown", 1));
    }

    @Test(expected=IllegalStateException.class)
    public void testMonitoringAlreadyStarted() throws OrekitException {
        CsvFileMonitor monitor = new CsvFileMonitor(out, "# ", ",", format, AbsoluteDate.J2000_EPOCH, 1.0);
        mass.register(monitor);
        position.register(monitor);
        velocity.register(monitor);
        AbsoluteDate t0 = AbsoluteDate.J2000_EPOCH.shiftedBy(10.0);
        mass.setDateAndValue(t0,     new double[] { 1000.0 });
        velocity.setDateAndValue(t0, new double[] { 44444.44, 55555.55, 66666.66 });
        position.setDateAndValue(t0, new double[] { 11111.11, 22222.22, 33333.33 });
        new BasicMonitorable("late monitorable", 1).register(monitor);
    }

    @Before
    public void setUp() {
        out      = new ByteArrayOutputStream();
        format   = new DecimalFormat("#0.00", new DecimalFormatSymbols(Locale.US));
        mass     = new BasicMonitorable("mass",     1);
        position = new BasicMonitorable("position", 3);
        velocity = new BasicMonitorable("velocity", 3);
    }

    @After
    public void tearDown() {
        Utils.setDataRoot("orekit-data");
        out      = null;
        format   = null;
        mass     = null;
        position = null;
        velocity = null;
    }

}
