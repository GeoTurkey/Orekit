/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.strategies.geo;

import eu.eumetsat.skat.control.AbstractSKControl;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.strategies.TunableManeuver;
import eu.eumetsat.skat.utils.SkatException;
import java.util.List;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.MathUtils;
import org.orekit.bodies.BodyShape;
import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.LocalOrbitalFrame;
import org.orekit.frames.LOFType;
import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.analytical.AdapterPropagator;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.PVCoordinatesProvider;
import org.orekit.utils.TimeStampedPVCoordinates;

/**
 * Station-keeping simultaneous control for inclination vector and longitude.
 * <p>
 * This control law schedules maneuvers that control simultaneously the inclination
 * vector and the longitude of the satellite. They are performed by rotating the
 * satellite around the yaw axis.
 * </p>
 * <p>
 * The maneuvers are triggered only by violations of the longitude control band,
 * not by violations of the inclination control circle.
 * </p>
 */
public class InclinationAndLongitude extends AbstractSKControl{
    
    /** Longitude control law. */
    private ParabolicLongitude parabolicLongitude;
    
    /** Inclination control law. */
    private InclinationVector inclinationVector;
    
    /** Associated step handler. */
    private final OrekitStepHandler stephandler;
    
    /** Iteration number. */
    private int iteration;
    
    /** Flag indicating whether maneuvers must be paired (2 maneuvers separated by 12 hours) or not. */
    private boolean isPairedManeuvers;
    
    /** Maximum number of maneuvers to set up in one cycle. */
    private final int maxManeuvers;
    
    /** Maximum yaw angle accepted in a maneuver. */
    private final double maxYaw = FastMath.toRadians(5.0);
    
    /** Yaw axis. */
    private final Vector3D yawAxis = new Vector3D(0.0,0.0,1.0);
    
    /** Maneuver models currently used */
    private TunableManeuver[] currentModels; 
    
    public InclinationAndLongitude(final String name, final String controlledName, final int controlledIndex,
                             final TunableManeuver[] model, int[][] yawFlipSequence, final double firstOffset,
                             final int maxManeuvers, final int orbitsSeparation,
                             final double referenceHx, final double referenceHy,
                             final double lEast, final double lWest,final BodyShape earth,
                             final double limitInclination, final double samplingStep, 
                             final double horizon, final boolean isPairedManeuvers) throws SkatException {
    
        super(name, model, yawFlipSequence, controlledName, controlledIndex, null, -1,
              FastMath.toDegrees(lWest), FastMath.toDegrees(MathUtils.normalizeAngle(lEast, lWest)),
              horizon * Constants.JULIAN_DAY);
        
        // Create a model for the longitude control of pure longitude manoeuvres
        final Vector3D direction               = new Vector3D(1.0,0.0,0.0);
        final TunableManeuver[] longitudeModel = new TunableManeuver[1];
        longitudeModel[0]                      = new TunableManeuver(model[0],direction);

        // Build the longitude and inclination control laws
        parabolicLongitude = new ParabolicLongitude(name, controlledName, controlledIndex, longitudeModel, 
                                                    yawFlipSequence, firstOffset, maxManeuvers, orbitsSeparation, 
                                                    lEast, lWest, samplingStep, horizon, earth);
        inclinationVector  = new InclinationVector(name, controlledName, controlledIndex, model, yawFlipSequence,
                                                   firstOffset, maxManeuvers, orbitsSeparation, referenceHx, 
                                                   referenceHy, limitInclination, samplingStep, horizon, isPairedManeuvers); 
        
        // Use longitude control step handler
        this.stephandler       = parabolicLongitude.getStepHandler();
        this.isPairedManeuvers = isPairedManeuvers;
        this.maxManeuvers      = maxManeuvers;
        
        if (this.isPairedManeuvers) {
            // Both models are used at the same time
            currentModels = models; 
        } else {
            currentModels = new TunableManeuver[1]; 
        }
        
    }
    
    /** {@inheritDoc} */
    public void initializeRun(final int iteration, final ScheduledManeuver[] maneuvers,
                              final Propagator propagator, final List<ScheduledManeuver> fixedManeuvers,
                              final AbsoluteDate start, final AbsoluteDate end) throws OrekitException
    {
        // Initialize longitude and inclination control laws
        parabolicLongitude.initializeRun(iteration, maneuvers, propagator, fixedManeuvers, start, end);
        inclinationVector.initializeRun(iteration, maneuvers, propagator, fixedManeuvers, start, end);
        this.iteration = iteration;
    }
    
    /** {@inheritDoc} */
    public ScheduledManeuver[] tuneManeuvers(final ScheduledManeuver[] tunables,
                                             final BoundedPropagator reference) throws OrekitException, SkatException
    {
        final ScheduledManeuver[] tuned;
        final AdapterPropagator adapterPropagator = new AdapterPropagator(reference);
        
        // Get the  Local Orbital Frame that considers the attitude change
        LocalOrbitalFrame myLOF = getLOF(adapterPropagator);
        
        if (iteration == 0)
        {
            // Compute first the inclination maneuvers and get them
            ScheduledManeuver[] dummyTunables = new ScheduledManeuver[0];
            final ScheduledManeuver[] tunedInclination = inclinationVector.computeManeuers(dummyTunables, adapterPropagator);
            
            // Get the DV of the longitude manoeuvres
            final double longitudeDV = parabolicLongitude.computeRequiredDV();
            
            if (!this.isPairedManeuvers) {
                // Update the maneuver model used
                currentModels[0] = tunedInclination[0].getModel().getName().equals(models[0].getName()) ? models[0] : models[1]; 
            }
            
            // Get the inclination + longitude maneuvers            
            tuned = distributeManeuversEvenly(tunedInclination, adapterPropagator, longitudeDV, 0.0,0.0);
        }
        else
        {
            // Compute longitude DV change
            final double longitudeDV = parabolicLongitude.computeDVChange();
            
            // Compute inclination DV and date change
            final InclinationVector.ManeuverChangedValues maneuverChangedValues = 
                    inclinationVector.computeManeuvresChange(tunables, tunables.length);
            
            // Get change apply changes to the inclination + longitude maneuvers
            tuned = distributeManeuversEvenly(tunables, adapterPropagator,  
                        longitudeDV, maneuverChangedValues.deltaV,maneuverChangedValues.deltaT);
            
        }
        
        // finalize propagator
        for (final ScheduledManeuver maneuver : tuned) {
            adapterPropagator.addEffect(new SmallManeuverAnalyticalModel(maneuver.getStateBefore(), (Frame) myLOF,
                                                                         maneuver.getDeltaV(),
                                                                         maneuver.getIsp()));        
        }
        
        return tuned;
    }
    
    /** {@inheritDoc} */
    public EventDetector getEventDetector() {
        return null;
    }

    /** {@inheritDoc} */
    public OrekitStepHandler getStepHandler() {
        return stephandler;
    }
    
    private ScheduledManeuver[] distributeManeuversEvenly(ScheduledManeuver[] maneuvers, AdapterPropagator adapterPropagator,
            final double longitudeDV, final double inclinationDV, final double dT)
    {
        if (!this.isPairedManeuvers) {
            // Just one model is used
            
            // Get a vector with all the DV of the maneuvers
            final Vector3D[] arrTotalDV = getTotalDV(maneuvers, longitudeDV, inclinationDV);
            final Vector3D totalDV = arrTotalDV[0];
            
            TunableManeuver model = getModel(arrTotalDV)[0];
            
            // Check if more maneuvers have to be added
            final int nManReq = (int) FastMath.ceil(totalDV.getNorm()/maneuvers[0].getModel().getCurrentDVSup());
            
            // Create equal maneuvers
            AbsoluteDate firstMan = maneuvers[0].getDate();
            final double separation = inclinationVector.getSeparation();
            ScheduledManeuver[] tuned = new ScheduledManeuver[FastMath.min(maxManeuvers,nManReq)];
            if (nManReq > maxManeuvers) {                
                // Schedule the maneuvers
                for (int i_man = 0; i_man < maxManeuvers; i_man++) {
                    tuned[i_man] = new ScheduledManeuver(model, firstMan.shiftedBy(i_man * separation),
                                                            new Vector3D(model.getCurrentDVSup(), model.getDirection()),
                                                            model.getCurrentThrust(), model.getCurrentISP(),
                                                            adapterPropagator, false);
                    tuned[i_man].setYawAngle(getYawAngle(tuned[i_man].getDeltaV(), currentModels[0]));
                }
                
            } else {    
                // Shift the non-saturated maneuvers
                firstMan = firstMan.shiftedBy(dT);
                final double DV = totalDV.getNorm()/nManReq;
                for (int i_man = 0; i_man < nManReq; i_man++) {
                    tuned[i_man] = new ScheduledManeuver(model, firstMan.shiftedBy(i_man * separation),
                                                            new Vector3D(DV, model.getDirection()),
                                                            model.getCurrentThrust(), model.getCurrentISP(),
                                                            adapterPropagator, false);
                }
            }
            return tuned;
            
        } else {
            // Two models are used
            
            // Get the AV required for the maneuvers
            final Vector3D[] totalDV = getTotalDV(maneuvers, longitudeDV, inclinationDV);
            final double maxPairedDV = currentModels[0].getCurrentDVSup()+currentModels[1].getCurrentDVSup();
            
            // Check the numbers of maneuvers required
            final int nPairsReq = (int) FastMath.ceil( (totalDV[0].getNorm()+totalDV[1].getNorm())/maxPairedDV);
            
            // Create the new models for the maneuvers
            TunableManeuver[] newModels = getModel(totalDV);
            
            // Create equal maneuvers by pairs
            AbsoluteDate firstMan        = maneuvers[0].getDate();
            final double separation      = inclinationVector.getSeparation();
            final int idx_firstManModel  = newModels[0].getName().equals(maneuvers[0].getModel().getName()) ? 0:1;
            final int idx_secManModel    = -1*idx_firstManModel+1; // The opposite one
            final TunableManeuver firstManModel  = newModels[idx_firstManModel];
            final TunableManeuver secondManModel = newModels[idx_secManModel]; 
            
            ScheduledManeuver[] tuned = new ScheduledManeuver[2*FastMath.min(maxManeuvers,nPairsReq)];
            if (nPairsReq > maxManeuvers) {
                
                
                for (int i_pair = 0; i_pair < maxManeuvers; i_pair++)
                {
                    tuned[2*i_pair] = new ScheduledManeuver(firstManModel, firstMan.shiftedBy(i_pair * separation),
                                                            new Vector3D(firstManModel.getCurrentDVSup(), firstManModel.getDirection()),
                                                            firstManModel.getCurrentThrust(), firstManModel.getCurrentISP(),
                                                            adapterPropagator, false);
                    
                    tuned[2*i_pair].setYawAngle(getYawAngle(tuned[2*i_pair].getDeltaV(), currentModels[idx_firstManModel]));
                    
                    tuned[2*i_pair+1] = new ScheduledManeuver(secondManModel, firstMan.shiftedBy(i_pair * separation+ Constants.JULIAN_DAY / 2),
                                                            new Vector3D(secondManModel.getCurrentDVSup(), secondManModel.getDirection()),
                                                            secondManModel.getCurrentThrust(), secondManModel.getCurrentISP(),
                                                            adapterPropagator, false);
                    
                    tuned[2*i_pair+1].setYawAngle(getYawAngle(tuned[2*i_pair+1].getDeltaV(), currentModels[idx_secManModel]));
                }
            } else {
                // Shift the non-saturated maneuvers
                firstMan = firstMan.shiftedBy(dT);
                final double firstDV  = totalDV[idx_firstManModel].getNorm()/nPairsReq;
                final double secondDV = totalDV[idx_secManModel].getNorm()/nPairsReq;
                
                for (int i_pair = 0; i_pair < maxManeuvers; i_pair++)
                {
                    tuned[2*i_pair] = new ScheduledManeuver(firstManModel, firstMan.shiftedBy(i_pair * separation),
                                                            new Vector3D(firstDV, firstManModel.getDirection()),
                                                            firstManModel.getCurrentThrust(), firstManModel.getCurrentISP(),
                                                            adapterPropagator, false);
                    
                    tuned[2*i_pair+1] = new ScheduledManeuver(secondManModel, firstMan.shiftedBy(i_pair * separation+ Constants.JULIAN_DAY / 2),
                                                            new Vector3D(secondDV, secondManModel.getDirection()),
                                                            secondManModel.getCurrentThrust(), secondManModel.getCurrentISP(),
                                                            adapterPropagator, false);
                }
            }
            return tuned;
        }
    }
    
    /** Compute the summed Delta V of all the input maneuvers, adding also 
     * the input longitudeDV and inclinationDV. If the maneuvers are paired, 
     * it returns two vectors. 
     * @param maneuvers     array with the maneuvers
     * @param longitudeDV   Delta V corresponding to longitude maneuvers that is 
     *                          not considered in the maneuvers array  
     * @param inclinationDV Delta V corresponding to inclination maneuvers that is 
     *                          not considered in the maneuvers array  
     * @return Change on Delta V
     */
    private Vector3D[] getTotalDV(final ScheduledManeuver[] maneuvers, final double longitudeDV, final double inclinationDV) {
        
        // Get a vector with all the DV of the maneuvers
        Vector3D[] totalDV = new Vector3D[currentModels.length];
        final double maxDV;
        if (this.isPairedManeuvers) {
            maxDV = currentModels[0].getCurrentDVSup()+currentModels[1].getCurrentDVSup();
        } else {
            maxDV = currentModels[0].getCurrentDVSup();
        }   

        for (int i_model = 0; i_model < currentModels.length; i_model++) {                
            totalDV[i_model] = new Vector3D(0.0,0.0,0.0);
            for (ScheduledManeuver maneuver : maneuvers)
            {
                if (maneuver.getModel().getName().equals(currentModels[i_model].getName()))
                {
                    totalDV[i_model] = totalDV[i_model].add(maneuver.getDeltaV());
                }
            }
            // inclinationDV > 0 if it follows the direction of the model
            final double incDVsign = FastMath.signum(currentModels[i_model].getDirection().getY());
            totalDV[i_model] = totalDV[i_model].add(
                    (new Vector3D(longitudeDV,inclinationDV*incDVsign,0.0)).scalarMultiply(currentModels[i_model].getCurrentDVSup()/maxDV));                
        }
        return totalDV;
    }
    
    /** Create a VNC Local Orbital Frame that considers the attitude modifications 
     * produced by the maneuvers added as effects to the adapter propagator.
     * @param adapterPropagator that stores the trajectory modified by the maneuvers scheduled     
     * @return Local orbital frame
     */
    private LocalOrbitalFrame getLOF(final AdapterPropagator adapterPropagator) {
        LocalOrbitalFrame adapterLOF = new LocalOrbitalFrame(FramesFactory.getEME2000(), LOFType.VNC, new PVCoordinatesProvider() {
            /** {@inheritDoc} */
                public TimeStampedPVCoordinates getPVCoordinates(AbsoluteDate date, Frame frame)
                    throws OrekitException {
                    return adapterPropagator.propagate(date).getPVCoordinates(frame);
                }
        }, "adapterLOF");
        return adapterLOF;
    }
    
    /** Returns the model of the maneuvers, which direction is equal to the one 
     * of the input vector. However, if the maneuver is saturated, the Y component 
     * of the direction is reduced (Delta V used to control inclination).
     * If yaw angle of the maneuver is greater than the maximum one, the maneuver
     * is performed with the maximum yaw angle
     * @param totalDV Delta V required by the maneuvers. If the maneuvers are paired,
     *                  this array contains two vectors.
     * @return model of the maneuvers.
     */
    private TunableManeuver[] getModel(Vector3D[] totalDV) {
        // Check the numbers of maneuvers or pairs of maneuvers required
        final int nbMan;
        final double maxDV;
        if (this.isPairedManeuvers) {
            maxDV = currentModels[0].getCurrentDVSup()+currentModels[1].getCurrentDVSup();
            nbMan = (int) FastMath.ceil( (totalDV[0].getNorm()+totalDV[1].getNorm())/maxDV);
        } else {
            maxDV = currentModels[0].getCurrentDVSup();            
            nbMan = (int) FastMath.ceil(totalDV[0].getNorm()/maxDV);
        }        

        // Create the new models for the maneuvers
        TunableManeuver[] newModels = new TunableManeuver[currentModels.length];
        for (int i_model = 0; i_model < currentModels.length; i_model++) { 
            if (nbMan > maxManeuvers) {
                // Compute the direction of the saturated maneuver preserving the AV of the longitude control
                final double incDVsign = FastMath.signum(currentModels[i_model].getDirection().getY());
                final double satVy     = incDVsign*FastMath.sqrt(FastMath.pow(currentModels[i_model].getCurrentDVSup()*maxManeuvers,2) 
                                                - FastMath.pow(totalDV[i_model].getX(),2) - FastMath.pow(totalDV[i_model].getZ(),2));
                final Vector3D satDir  = new Vector3D(totalDV[i_model].getX(),satVy,totalDV[i_model].getZ());
                
                // Check if the yaw angle is with in range
                if (FastMath.abs(getYawAngle(satDir,currentModels[i_model])) > maxYaw) {
                     newModels[i_model]     = new TunableManeuver(currentModels[i_model],getMaxYawManeuver(satDir,currentModels[i_model]));
                } else {
                    // Use the direction of the saturated maneuver
                    newModels[i_model]     = new TunableManeuver(currentModels[i_model],satDir);
                }

            } else {
                // Check if the yaw angle is with in range
                if (FastMath.abs(getYawAngle(totalDV[i_model],currentModels[i_model])) > maxYaw) {
                     newModels[i_model] = new TunableManeuver(currentModels[i_model],getMaxYawManeuver(totalDV[i_model],currentModels[i_model]));
                } else {
                    // Use the direction of totalDV
                    newModels[i_model] = new TunableManeuver(currentModels[i_model],totalDV[i_model]);
                }
            }
        }        
        return newModels;
    }
    
    /** Compute the signed yaw angle between the direction of reference model and 
     * the input vector.
     * @param vector
     * @param referenceModel 
     * @return signed yaw angle.
     */
    private double getYawAngle(final Vector3D vector, final TunableManeuver referenceModel) {
        // Remove the component along the yaw axis 
        final Vector3D directionYawPerp = vector.subtract(yawAxis.scalarMultiply(vector.dotProduct(yawAxis)));
        final Vector3D reference = referenceModel.getDirection().subtract(yawAxis.scalarMultiply(referenceModel.getDirection().dotProduct(yawAxis)));
        final double yawAngle = Vector3D.angle(directionYawPerp, reference);
        
        // Add the sing
        final double signum = FastMath.signum(reference.crossProduct(directionYawPerp).dotProduct(yawAxis));
        return signum*yawAngle;
    }
    
    /** Return the input manDirection rotated in order to form the maximum angle
     * with the direction of the reference model.
     * @param manDirection
     * @param referenceModel 
     * @return rotated manDiretion.
     */
    private Vector3D getMaxYawManeuver(final Vector3D manDirection, final TunableManeuver referenceModel) {
        // Rotation of value maxYaw around the yawAxis
        final double yawAngle = getYawAngle(manDirection,referenceModel);
        final double rotAngle = FastMath.abs(yawAngle)-maxYaw;
        final Rotation rotation = new Rotation(yawAxis, -rotAngle*FastMath.signum(yawAngle));
        return rotation.applyTo(manDirection);
    }
}
