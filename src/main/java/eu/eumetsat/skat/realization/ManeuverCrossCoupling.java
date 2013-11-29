/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.forces.maneuvers.SmallManeuverAnalyticalModel;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;
import eu.eumetsat.skat.strategies.ScheduledManeuver;
import eu.eumetsat.skat.utils.SkatException;
import eu.eumetsat.skat.utils.SkatMessages;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;

/**
 * Class for computing cross-coupling in maneuvers.
 * <p>
 * Cross-coupling is due to deflection of the exhaust from
 * thrusters by spacecraft parts (solar arrays, antennas ...).
 * It is modeled by a small rotation of the thrust direction.
 * </p>
 * <p>
 * From an engineering point of view, cross-coupling is often
 * defined as a percentage or ratio with respect to the original
 * thrust. So the user has to define a nominal direction for the
 * thrust in spacecraft frame (say Y axis for example), a coupling
 * direction in spacecraft frame (say X axis for example) and a
 * coupling ratio r. From these defining elements, the coupling
 * model will be a rotation around axis -Z with angle asin(r).
 * </p>
 * <p>
 * Coupling factor r has the follows the expression: 
 * <ul>
 * r = r0 + secularVariation * (t - t0secular) + amplitude * Sin[2*PI * ( t - t0harmonic ) / period], where:
 * <ul>
 * <li>secularVariation is given in 1/years.</li>
 * <li>period is given in days.</li>
 * </ul>
 * </ul>
 * If standardDeviation is not zero, an uncertainty factor is added
 * to r. 
 * </p>
 * @author Luc Maisonobe
 */
public class ManeuverCrossCoupling implements ScenarioComponent {

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Name of maneuvers to which this component applies. */
    private final String name;

    /** Coupling components. */
    private final Vector3D nominalDirection;
    private final Vector3D couplingDirection1, couplingDirection2;
    private final double constantCouplingRatio1, constantCouplingRatio2;
    final double secularVariation1, secularVariation2; 
    final AbsoluteDate secularZero1, secularZero2;
    final double period1, period2; 
    final AbsoluteDate harmonicZero1, harmonicZero2;                                 
    final double amplitude1, amplitude2;

    /** Standard deviation of the multiplying uncertainty factor. */
    private final double standardDeviation1, standardDeviation2;

    /** Random generator to use for evaluating the uncertainty factor. */
    private final RandomGenerator generator;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param name name of maneuvers to which this component applies
     * @param nominalDirection nominal direction of the thrust in spacecraft frame
     * @param generator random generator to use for evaluating the uncertainty factor
     * @param standardDeviation1 standard deviation of the multiplying uncertainty factor
     * @param couplingDirection1 coupling direction in spacecraft frame
     * @param constantCouplingRatio1 constant term of the coupling ratio
     * @param secularVariation1 secular derivative in 1/years 
     * @param secularZero1 date when the secular variation is zero     
     * @param period1 period of the harmonic term in days. If its value is zero 
     * @param harmonicZero1 date when the harmonic term is zero
     * @param amplitude1 amplitude of the harmonic term
     * (minor than 1e-12), amplitude is set to zero.
     * (for example 0.05 for a 5% uncertainty)     
     * @param standardDeviation2 second coupling standard deviation of the multiplying uncertainty factor
     * @param couplingDirection2 second coupling direction in spacecraft frame
     * @param constantCouplingRatio2 second coupling constant term of the coupling ratio
     * @param secularVariation2 second coupling secular derivative in 1/years 
     * @param secularZero2 second coupling date when the secular variation is zero     
     * @param period2 second coupling period of the harmonic term in days. If its value is zero 
     * @param harmonicZero2 second coupling date when the harmonic term is zero
     * @param amplitude2 second coupling amplitude of the harmonic term
     * (minor than 1e-12), amplitude is set to zero.
     * (for example 0.05 for a 5% uncertainty)     
     * @exception IllegalArgumentException if the constant term of the coupling 
     * ratio is not between -1 and 1 or if coupling direction is aligned with
     * nominal thrust direction
     */
    public ManeuverCrossCoupling(final int[] spacecraftIndices, final String name,
    		final Vector3D nominalDirection, 
    		final RandomGenerator generator,
    		final double standardDeviation1, final Vector3D couplingDirection1, 
    		final double constantCouplingRatio1, final double secularVariation1, final AbsoluteDate secularZero1, 
    		final double period1, final AbsoluteDate harmonicZero1, final double amplitude1, 
    		final double standardDeviation2, final Vector3D couplingDirection2, 
    		final double constantCouplingRatio2, final double secularVariation2, final AbsoluteDate secularZero2, 
    		final double period2, final AbsoluteDate harmonicZero2, final double amplitude2)
        throws IllegalArgumentException {
        if (constantCouplingRatio1 < -1 || constantCouplingRatio1 > 1) {
            throw SkatException.createIllegalArgumentException(SkatMessages.WRONG_COUPLING,
                                                               constantCouplingRatio1);
        }
        if (Vector3D.angle(couplingDirection1, nominalDirection) < 0.1) {
            throw SkatException.createIllegalArgumentException(SkatMessages.ALIGNED_COUPLING_AXES,
                                                               couplingDirection1.getX(),
                                                               couplingDirection1.getY(),
                                                               couplingDirection1.getZ());
        }
        if (constantCouplingRatio2 < -1 || constantCouplingRatio2 > 1) {
            throw SkatException.createIllegalArgumentException(SkatMessages.WRONG_COUPLING,
                                                               constantCouplingRatio1);
        }
        if (Vector3D.angle(couplingDirection2, nominalDirection) < 0.1) {
            throw SkatException.createIllegalArgumentException(SkatMessages.ALIGNED_COUPLING_AXES,
                                                               couplingDirection2.getX(),
                                                               couplingDirection2.getY(),
                                                               couplingDirection2.getZ());
        }
        this.spacecraftIndices      = spacecraftIndices.clone();
        this.name                   = name;
        this.nominalDirection       = nominalDirection;
        this.standardDeviation1     = standardDeviation1;
        this.standardDeviation2     = standardDeviation2;
        this.generator              = generator;
        this.couplingDirection1     = couplingDirection1;
        this.couplingDirection2     = couplingDirection2;
        this.constantCouplingRatio1 = constantCouplingRatio1;
        this.constantCouplingRatio2 = constantCouplingRatio2;
        this.secularVariation1      = secularVariation1; 
        this.secularVariation2      = secularVariation2; 
        this.secularZero1           = secularZero1;
        this.secularZero2           = secularZero2;
        this.harmonicZero1          = harmonicZero1; 
        this.harmonicZero2          = harmonicZero2; 
        if (FastMath.abs(period1) < 1e-12 ) {
            this.period1    = 1.0;
            this.amplitude1 = 0.0;            
        } else {
            this.period1    = period1; 
            this.amplitude1 = amplitude1;
        }        
        if (FastMath.abs(period2) < 1e-12 ) {
            this.period2    = 1.0;
            this.amplitude2 = 0.0;            
        } else {
            this.period2    = period2; 
            this.amplitude2 = amplitude2;
        }        
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException, SkatException {

        ScenarioState[] updated = originals.clone();

        for (int i = 0; i < spacecraftIndices.length; ++i) {

            // select the current spacecraft affected by this component
            final int index = spacecraftIndices[i];
            final List<ScheduledManeuver> rawManeuvers = originals[index].getManeuvers();
            if (rawManeuvers == null) {
                throw new SkatException(SkatMessages.NO_MANEUVERS_STATE,
                                        originals[index].getName(), originals[index].getCyclesNumber());
            }

            // prepare a list for holding the modified maneuvers
            List<ScheduledManeuver> modified = new ArrayList<ScheduledManeuver>();

            // modify the maneuvers
            for (final ScheduledManeuver maneuver : rawManeuvers) {
                if (maneuver.getName().equals(name)) {
                    // the maneuver is affected by the coupling with some uncertainty
                    final double errorFactor1   = 1.0 + standardDeviation1 * generator.nextGaussian();
                    final double couplingRatio1 = getCouplingRatio(maneuver.getDate(), secularZero1, constantCouplingRatio1, 
                    		                                       secularVariation1, harmonicZero1, period1, amplitude1);
                    final Rotation coupling1    = new Rotation(Vector3D.crossProduct(nominalDirection, couplingDirection1),
                                                               FastMath.asin(errorFactor1 * couplingRatio1));
                    
                    final double errorFactor2   = 1.0 + standardDeviation2 * generator.nextGaussian();
                    final double couplingRatio2 = getCouplingRatio(maneuver.getDate(), secularZero2, constantCouplingRatio2, 
                                                                   secularVariation2, harmonicZero2, period2, amplitude2);
                    final Rotation coupling2    = new Rotation(Vector3D.crossProduct(nominalDirection, couplingDirection2),
                                                               FastMath.asin(errorFactor2 * couplingRatio2));
                    
                    final Rotation coupling   = coupling1.applyTo(coupling2);
                    final ScheduledManeuver m = new ScheduledManeuver(maneuver.getModel(), maneuver.getDate(),
                                                                      coupling.applyTo(maneuver.getDeltaV()),
                                                                      maneuver.getThrust(),
                                                                      maneuver.getIsp(), maneuver.getTrajectory(),
                                                                      false);
                    maneuver.getTrajectory().addEffect(new SmallManeuverAnalyticalModel(maneuver.getStateBefore(),
                                                                                        maneuver.getDeltaV().negate(),
                                                                                        maneuver.getIsp()));
                    m.getTrajectory().addEffect(new SmallManeuverAnalyticalModel(m.getStateBefore(),
                                                                                 m.getDeltaV(),
                                                                                 m.getIsp()));
                    modified.add(m);
                } else {
                    // the maneuver is immune to the error
                    modified.add(maneuver);
                }
            }

            // update the state
            updated[index] = originals[index].updateManeuvers(modified);

        }

        // return an updated states
        return updated;

    }
    
    /** Compute cross-coupling ratio at a certain date
     * @param date date when the cross coupling is computed
     * @param secularZero date when the linear term is zero
     * @param constantCouplingRatio constant term
     * @param secularVariation secular derivative
     * @param harmonicZero date when the harmonic term is zero
     * @param period of the harmonic term
     * @param amplitude of the harmonic term
     * @return couplingRatio cross-coupling ratio
     * @exception IllegalArgumentException if coupling ratio is not between -1 and 1
     */
    private double getCouplingRatio(final AbsoluteDate date, 
    		                        final AbsoluteDate secularZero,  final double constantCouplingRatio, final double secularVariation, 
    		                        final AbsoluteDate harmonicZero, final double period, final double amplitude)
    		                        throws IllegalArgumentException {
        final double deltaTsecular = date.durationFrom(secularZero);
        final double deltaPhi1     = 2 * FastMath.PI * date.durationFrom(harmonicZero) / (period * Constants.JULIAN_DAY);
        final double couplingRatio = constantCouplingRatio + secularVariation * deltaTsecular / Constants.JULIAN_YEAR +
                                      amplitude * FastMath.sin(deltaPhi1);
        
         if (couplingRatio < -1 || couplingRatio > 1) {
            throw SkatException.createIllegalArgumentException(SkatMessages.WRONG_COUPLING,
            		couplingRatio);
        }
        
        return couplingRatio;
    }

}
