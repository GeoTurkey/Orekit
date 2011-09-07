/* Copyright 2011 Eumetsat */

package eu.eumetsat.skat;

import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.AbstractDetector;

public class EscapeDetector extends AbstractDetector {

    /** Serializable UID. */
    private static final long serialVersionUID = 7509299531915475688L;

    private BodyShape earth;
    private double east;
    private double west;

    public EscapeDetector(double maxCheck, double threshold, BodyShape earth, double east, double west) {
        super(maxCheck, threshold);
        this.earth = earth;
        this.east = east;
        this.west = west;
    }

    public double g(SpacecraftState s) throws OrekitException {

        // compute position in Earth frame
        Vector3D position = s.getPVCoordinates(earth.getBodyFrame()).getPosition();

        // convert to latitude/longitude/altitude
        GeodeticPoint gp = earth.transform(position, earth.getBodyFrame(), s.getDate());

        // compute a function that is negative in the longitude slot,
        // zero at slot boundaries, and positive outside slot
        // thus, events are triggered at BOTH East and West escapes
        return (gp.getLongitude() - east) * (gp.getLongitude() - west);

    }

    public int eventOccurred(SpacecraftState s, boolean increasing) {
        return STOP;
    }

}
