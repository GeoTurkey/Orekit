/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

import java.io.ByteArrayOutputStream;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.text.Format;
import java.util.Locale;

import junit.framework.Assert;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.orekit.Utils;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;


public class CsvFileMonitorTest {

    private ByteArrayOutputStream out;
    private Format format;
    private MonitorableSKData mass;
    private MonitorableSKData position;
    private MonitorableSKData velocity;

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
        Assert.assertEquals("# column 2: spacecraft mass",        lines[3]);
        Assert.assertEquals("# column 3: position eme2000[0]", lines[4]);
        Assert.assertEquals("# column 4: position eme2000[1]", lines[5]);
        Assert.assertEquals("# column 5: position eme2000[2]", lines[6]);
        Assert.assertEquals("# column 6: velocity eme2000[0]", lines[7]);
        Assert.assertEquals("# column 7: velocity eme2000[1]", lines[8]);
        Assert.assertEquals("# column 8: velocity eme2000[2]", lines[9]);
    }

    @Test
    public void testSmallNominalCase() throws OrekitException {
        CsvFileMonitor monitor = new CsvFileMonitor(out, "# ", ",", format, AbsoluteDate.J2000_EPOCH, 1.0);
        mass.register(monitor);
        position.register(monitor);
        velocity.register(monitor);
        Snapshot[] snapshots0 = createSnapshots(10.0, 1000.0, 11111.11, 22222.22, 33333.33, 44444.44, 55555.55, 66666.66);
        mass.update(snapshots0);
        velocity.update(snapshots0);
        position.update(snapshots0);
        Snapshot[] snapshots1 = createSnapshots(20.0, 1000.0, 44444.44, 55555.55, 66666.66, 11111.11, 22222.22, 33333.33);
        mass.update(snapshots1);
        velocity.update(snapshots1);
        position.update(snapshots1);
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
        Snapshot[] snapshots0 = createSnapshots(10.0, 1000.0, 11111.11, 22222.22, 33333.33, 44444.44, 55555.55, 66666.66);
        mass.update(snapshots0);
        velocity.update(snapshots0);
        position.update(snapshots0);
        Snapshot[] snapshots1 = createSnapshots(10.000008, 1000.0, 11111.11, 22222.22, 33333.33, 77777.77, 88888.88, 99999.99);
        velocity.update(snapshots1);
        Snapshot[] snapshots2 = createSnapshots(20.0, 1000.0, 44444.44, 55555.55, 66666.66, 11111.11, 22222.22, 33333.33);
        velocity.update(snapshots2);
        position.update(snapshots2);
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
        new Monitorable(){

            public void register(Monitor monitor) {
                monitor.startMonitoring(this);
            }

            public String getName() {
                return mass.getName();
            }

            public AbsoluteDate getDate() {
                return mass.getDate();
            }

            public double[] getValue() {
                return mass.getValue();
            }
            
        }.register(monitor);
    }

    @Test(expected=IllegalArgumentException.class)
    public void testNotMonitored() throws OrekitException {
        CsvFileMonitor monitor = new CsvFileMonitor(out, "# ", ",", format, AbsoluteDate.J2000_EPOCH, 1.0);
        monitor.valueChanged(MonitorableSKData.CYCLES_NUMBER);
    }

    @Test(expected=IllegalStateException.class)
    public void testMonitoringAlreadyStarted() throws OrekitException {
        CsvFileMonitor monitor = new CsvFileMonitor(out, "# ", ",", format, AbsoluteDate.J2000_EPOCH, 1.0);
        mass.register(monitor);
        position.register(monitor);
        velocity.register(monitor);
        Snapshot[] snapshots = createSnapshots(10.0, 1000.0, 44444.44, 55555.55, 66666.66, 11111.11, 22222.22, 33333.33);
        mass.update(snapshots);
        velocity.update(snapshots);
        position.update(snapshots);
        MonitorableSKData.CYCLES_NUMBER.register(monitor);
    }

    private Snapshot[] createSnapshots(final double dt, final double mass,
                                       final double x,  final double y,  final double z,
                                       final double vx, final double vy, final double vz)
        throws OrekitException {
        PVCoordinates pv = new PVCoordinates(new Vector3D(x, y, z),
                                             new Vector3D(vx, vy, vz));
        Orbit orbit = new CartesianOrbit(pv, FramesFactory.getEME2000(),
                                         AbsoluteDate.J2000_EPOCH.shiftedBy(dt * Constants.JULIAN_DAY),
                                         Constants.EIGEN5C_EARTH_MU);
        SpacecraftState state = new SpacecraftState(orbit, mass);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               FramesFactory.getITRF2008());
        return new Snapshot[] { new Snapshot(state, 0, 0.0, 0, 0.0, 0.0, 1, earth) };
    }

    @Before
    public void setUp() {
        out      = new ByteArrayOutputStream();
        format   = new DecimalFormat("#0.00", new DecimalFormatSymbols(Locale.US));
        mass     = MonitorableSKData.SPACECRAFT_MASS;
        position = MonitorableSKData.POSITION_EME2000;
        velocity = MonitorableSKData.VELOCITY_EME2000;
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
