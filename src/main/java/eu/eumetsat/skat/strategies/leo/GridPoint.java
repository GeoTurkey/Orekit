/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.leo;


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

    /** Latitude. */
    private final double latitude;

    /** Longitude. */
    private final double longitude;

    /** If true, reference point is considered when crossing the specified latitude from South to North. */
    private boolean ascending;

    /** Simple constructor.
     * @param timeOffset time offset since some arbitrary cycle reference
     * @param latitude latitude
     * @param longitude longitude
     * @param ascending if true, reference point is considered when crossing
     * the specified latitude from south to north
     */
    public GridPoint(final double timeOffset, final double latitude, final double longitude,
                     final boolean ascending) {
        this.timeOffset = timeOffset;
        this.latitude   = latitude;
        this.longitude  = longitude;
        this.ascending  = ascending;
    }

    /** Get time offset.
     * @return time offset
     */
    public double getTimeOffset() {
        return timeOffset;
    }

    /** Get latitude.
     * @return latitude
     */
    public double getLatitude() {
        return latitude;
    }

    /** Get longitude.
     * @return longitude
     */
    public double getLongitude() {
        return longitude;
    }

    /** Get ascending indicator.
     * @return ascending indicator
     */
    public boolean isAscending() {
        return ascending;
    }

}
