package eu.eumetsat.skat.realization;


import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.attitudes.Attitude;
import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.EventDetector;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinates;

/** Mass consumer model.
 * <p>This class implements an instantaneous mass decreasement that 
 * can be added to a propagator as an {@link org.orekit.propagation.events.EventDetector
 * EventDetector}.This event can be provided to
 * any {@link org.orekit.propagation.Propagator Propagator}.</p> 
 * <p>The mass decreasement is triggered when an underlying event generates a
 * {@link org.orekit.propagation.events.EventDetector.Action#STOP STOP} event,
 * in which case this class will generate a {@link
 * org.orekit.propagation.events.EventDetector.Action#RESET_STATE RESET_STATE}
 * event (the stop event from the underlying object is therefore filtered out).</p>
 * <p> The mass decreasement is computed with the values of @param deltaVSat 
 * and @param isp.</p>
 * @see org.orekit.propagation.Propagator#addEventDetector(EventDetector)
 */
public class ImpulseManeuverNoIsp extends ImpulseManeuver {

    /**
	 * 
	 */
	private static final long serialVersionUID = -620621038585094163L;
	private double deltaMass;
	private Vector3D deltaVSat;

	/** Build a new instance.
     * @param trigger triggering event
     * @param deltaMass satellite mass decrease
     */
    public ImpulseManeuverNoIsp(EventDetector trigger, Vector3D deltaV, double deltaMass) {
		super(trigger, new Vector3D(0.0,0.0,0.0), 0.0);
		
		this.deltaMass = deltaMass;
		this.deltaVSat = deltaV;
	}

	/** {@inheritDoc} */
    @Override
    public SpacecraftState resetState(final SpacecraftState oldState)
        throws OrekitException {   
    	
        final AbsoluteDate date = oldState.getDate();
        final Attitude attitude = oldState.getAttitude();

        // convert velocity increment to inertial frame
        final Vector3D deltaV = attitude.getRotation().applyInverseTo(deltaVSat);

        // apply increment to position/velocity
        final PVCoordinates oldPV     = oldState.getPVCoordinates();
        final PVCoordinates newPV     = new PVCoordinates(oldPV.getPosition(), oldPV.getVelocity().add(deltaV));
        final CartesianOrbit newOrbit = new CartesianOrbit(newPV, oldState.getFrame(), date, oldState.getMu());
        final double newMass          = oldState.getMass() - this.deltaMass;
    	
    	return new SpacecraftState(newOrbit, newMass);
   }
}