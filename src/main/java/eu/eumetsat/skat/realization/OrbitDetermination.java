/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.realization;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.random.CorrelatedRandomVectorGenerator;
import org.apache.commons.math3.random.GaussianRandomGenerator;
import org.apache.commons.math3.random.RandomGenerator;
import org.orekit.errors.OrekitException;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.SpacecraftState;

import eu.eumetsat.skat.scenario.ScenarioComponent;
import eu.eumetsat.skat.scenario.ScenarioState;

/**
 * Class for orbit-determination simulation.
 * <p>
 * This class simulate orbit determination. It starts from the real orbit
 * and compute an estimated orbit by adding errors. The added error
 * vector is Gaussian, zero-mean and its covariance is specified by a
 * matrix set at construction time.
 * </p>
 * @author Luc Maisonobe
 */
public class OrbitDetermination implements ScenarioComponent {

    /** Indices of the spacecrafts managed by this component. */
    private final int[] spacecraftIndices;

    /** Vector generator. */
    private final CorrelatedRandomVectorGenerator generator;

    /** Standard deviation. */
    private final double[] standardDeviation;

    /** Orbit type used in the covariance  matrix. */
    private final OrbitType orbitType;

    /** Position angle used in the covariance matrix. */
    private final PositionAngle positionAngle;

    /** Simple constructor.
     * @param spacecraftIndices indices of the spacecrafts managed by this component
     * @param correlation correlation matrix to use for generating
     * orbit determination errors
     * @param standardDeviation standard deviation of the error
     * @param orbitType orbit type used in the covariance  matrix
     * @param positionAngle position angle used in the covariance matrix
     * @param small Diagonal elements threshold under which  column are
     * considered to be dependent on previous ones and are discarded
     * @param random raw random generator
     */
    public OrbitDetermination(final int[] spacecraftIndices,
                              final RealMatrix correlation,
                              final double[] standardDeviation,
                              final OrbitType orbitType,
                              final PositionAngle positionAngle,
                              final double small,
                              final RandomGenerator random) {
        this.spacecraftIndices = spacecraftIndices.clone();
        final double[] zeroMean = new double[correlation.getRowDimension()];
        generator = new CorrelatedRandomVectorGenerator(zeroMean, correlation, small,
                                                        new GaussianRandomGenerator(random));
        this.standardDeviation = standardDeviation.clone();
        this.orbitType         = orbitType;
        this.positionAngle     = positionAngle;
    }

    /** {@inheritDoc} */
    public ScenarioState[] updateStates(final ScenarioState[] originals)
        throws OrekitException {

        ScenarioState[] updated = originals.clone();

        for (int i = 0; i < spacecraftIndices.length; ++i) {

            // select the current spacecraft affected by this component
            final int index = spacecraftIndices[i];

            // get the real state vector
            final SpacecraftState realState = originals[index].getRealState();
            final Orbit realOrbit           = orbitType.convertType(realState.getOrbit());
            final double[] orbitArray       = new double[6];
            orbitType.mapOrbitToArray(realOrbit, positionAngle, orbitArray);

            // add correlated random error vector
            final double[] errorVector = generator.nextVector();
            for (int j = 0; j < orbitArray.length; ++j) {
                orbitArray[j] += standardDeviation[j] * errorVector[j];
            }

            // create the estimated state vector
            final Orbit randomizedOrbit =
                    orbitType.mapArrayToOrbit(orbitArray, positionAngle, realOrbit.getDate(),
                                              realOrbit.getMu(), realOrbit.getFrame());

            // convert it to the same type as original one
            // (which may be different from covariance matrix orbit type)
            final OrbitType originalType = realState.getOrbit().getType();
            final SpacecraftState estimatedState =
                    new SpacecraftState(originalType.convertType(randomizedOrbit),
                                        realState.getAttitude(),
                                        realState.getMass());

            // update the state
            updated[index] = originals[index].updateEstimatedState(estimatedState);
        }

        // return the updated states
        return updated;

    }

}
