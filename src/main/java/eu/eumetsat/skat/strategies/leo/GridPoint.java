/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;


/**
 * Reference grid point for phased Low Earth Orbit.
 * <p>
 * This class is a simple container.
 * </p>
 * @author Luc Maisonobe
 */
public class GridPoint {

    /** Time offset since some arbitrary cycle reference. */
    private final double timeOffset;

    /** Point as a geodetic point. */
    private final GeodeticPoint geodetic;

    /** Point as a Cartesian point. */
    private final Vector3D cartesian;

    /** If true, reference point is considered when crossing the specified latitude from South to North. */
    private boolean ascending;

    /** Simple constructor.
     * @param timeOffset time offset since some arbitrary cycle reference
     * @param latitude latitude
     * @param longitude longitude
     * @param earth Earth model
     * @param ascending if true, reference point is considered when crossing
     * the specified latitude from south to north
     */
    public GridPoint(final double timeOffset, final double latitude, final double longitude,
                     final OneAxisEllipsoid earth, final boolean ascending) {
        this.timeOffset = timeOffset;
        this.geodetic   = new GeodeticPoint(latitude, longitude, 0.0);
        this.cartesian  = earth.transform(geodetic);
        this.ascending  = ascending;
    }

    /** Get time offset.
     * @return time offset
     */
    public double getTimeOffset() {
        return timeOffset;
    }

    /** Get point at Earth surface.
     * @return point at Earth surface
     */
    public GeodeticPoint getGeodeticPoint() {
        return geodetic;
    }

    /** Get point at Earth surface.
     * @return point at Earth surface
     */
    public Vector3D getCartesianPoint() {
        return cartesian;
    }

    /** Get ascending indicator.
     * @return ascending indicator
     */
    public boolean isAscending() {
        return ascending;
    }

}
