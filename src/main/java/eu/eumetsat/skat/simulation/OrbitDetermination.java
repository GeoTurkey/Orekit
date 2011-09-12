package eu.eumetsat.skat.simulation;

import org.apache.commons.math.linear.RealMatrix;
import org.apache.commons.math.random.CorrelatedRandomVectorGenerator;
import org.apache.commons.math.random.GaussianRandomGenerator;
import org.apache.commons.math.random.RandomGenerator;
import org.orekit.errors.OrekitException;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.SpacecraftState;
import org.orekit.time.AbsoluteDate;

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

    /** Vector generator. */
    private final CorrelatedRandomVectorGenerator generator;

    /** Position angle used in the covariance matrix. */
    private final PositionAngle positionAngle;

    /** Simple constructor.
     * @param covariance covariance matrix to use for generating
     * orbit determination errors
     * @param positionAngle position angle used in the covariance matrix
     * @param small Diagonal elements threshold under which  column are
     * considered to be dependent on previous ones and are discarded
     * @param random raw random generator
     */
    public OrbitDetermination(final RealMatrix covariance,
                              final PositionAngle positionAngle,
                              final double small,
                              final RandomGenerator random) {
        final double[] zeroMean = new double[covariance.getRowDimension()];
        generator = new CorrelatedRandomVectorGenerator(zeroMean, covariance, small,
                                                        new GaussianRandomGenerator(random));
        this.positionAngle = positionAngle;
    }

    /** {@inheritDoc} */
    public ScenarioState apply(final ScenarioState original, AbsoluteDate target)
        throws OrekitException {

        // get the real state vector
        final SpacecraftState realState = original.getRealState();
        final Orbit realOrbit           = original.getRealState().getOrbit();
        final OrbitType type            = realOrbit.getType();
        final double[] orbitArray       = new double[6];
        type.mapOrbitToArray(realOrbit, positionAngle, orbitArray);

        // add correlated random error vector
        final double[] errorVector = generator.nextVector();
        for (int i = 0; i < orbitArray.length; ++i) {
            orbitArray[i] += errorVector[i];
        }

        // create the estimated state vector
        final Orbit estimatedOrbit =
                type.mapArrayToOrbit(orbitArray, positionAngle, realOrbit.getDate(),
                                     realOrbit.getMu(), realOrbit.getFrame());
        final SpacecraftState estimatedState =
                new SpacecraftState(estimatedOrbit, realState.getAttitude(),
                                    realState.getMass());

        return new ScenarioState(realState, estimatedState, original.getManeuvers());

    }

}
