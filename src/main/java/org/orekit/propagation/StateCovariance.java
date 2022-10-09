/* Copyright 2002-2022 CS GROUP
 * Licensed to CS GROUP (CS) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * CS licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.orekit.propagation;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitMessages;
import org.orekit.frames.Frame;
import org.orekit.frames.LOFType;
import org.orekit.frames.Transform;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeStamped;
import org.orekit.utils.CartesianDerivativesFilter;

/** This class is the representation of a covariance matrix at a given date.
 * <p>
 * It is possible to change the covariance frame by using the
 * {@link #changeCovarianceFrame(Orbit, Frame)} or {@link #changeCovarianceFrame(Orbit, LOFType)} method.
 * These methods are based on Equations (18) and (20) of <i>Covariance Transformations for Satellite
 * Flight Dynamics Operations</i> by David A. SVallado.
 * <p>
 * Finally, covariance orbit type can be changed using the
 * {@link #changeCovarianceType(Orbit, OrbitType, PositionAngle) method.
 * </p>
 * @author Bryan Cazabonne
 * @author Vincent Cucchietti
 * @since 11.3
 */
public class StateCovariance implements TimeStamped {

    /** State dimension. */
    private static final int STATE_DIMENSION = 6;

    /** Default position angle for covariance expressed in cartesian elements. */
    private static final PositionAngle DEFAULT_POSITION_ANGLE = PositionAngle.TRUE;

    /** Orbital covariance [6x6]. */
    private final RealMatrix orbitalCovariance;

    /** Covariance frame (can be null if lofType is defined). */
    private final Frame frame;

    /** Covariance LOF type (can be null if frame is defined). */
    private final LOFType lofType;

    /** Covariance epoch. */
    private final AbsoluteDate epoch;

    /** Covariance orbit type. */
    private final OrbitType orbitType;

    /** Covariance position angle type (not used if orbitType is CARTESIAN). */
    private final PositionAngle angleType;

    /**
     * Constructor.
     * @param orbitalCovariance 6x6 orbital parameters covariance
     * @param epoch epoch of the covariance
     * @param lofType covariance LOF type
     */
    public StateCovariance(final RealMatrix orbitalCovariance, final AbsoluteDate epoch, final LOFType lofType) {
        this(orbitalCovariance, epoch, null, lofType, OrbitType.CARTESIAN, DEFAULT_POSITION_ANGLE);
    }

    /**
     * Constructor.
     * @param orbitalCovariance 6x6 orbital parameters covariance
     * @param epoch epoch of the covariance
     * @param covarianceFrame covariance frame (inertial of Earth fixed)
     * @param orbitType orbit type of the covariance
     * @param angleType position angle type of the covariance
     *        (not used if orbitType is CARTESIAN)
     */
    public StateCovariance(final RealMatrix orbitalCovariance, final AbsoluteDate epoch,
                           final Frame covarianceFrame,
                           final OrbitType orbitType, final PositionAngle angleType) {
        this(orbitalCovariance, epoch, covarianceFrame, null, orbitType, angleType);
    }

    /**
     * Private constructor.
     * @param orbitalCovariance 6x6 orbital parameters covariance
     * @param epoch epoch of the covariance
     * @param covarianceFrame covariance frame (inertial of Earth fixed)
     * @param lofType covariance LOF type
     * @param orbitType orbit type of the covariance
     * @param angleType position angle type of the covariance
     *        (not used if orbitType is CARTESIAN)
     */
    private StateCovariance(final RealMatrix orbitalCovariance, final AbsoluteDate epoch,
                            final Frame covarianceFrame, final LOFType lofType,
                            final OrbitType orbitType, final PositionAngle angleType) {
        this.orbitalCovariance = orbitalCovariance;
        this.epoch = epoch;
        this.frame = covarianceFrame;
        this.lofType = lofType;
        this.orbitType = orbitType;
        this.angleType = angleType;
    }

    /** {@inheritDoc}. */
    @Override
    public AbsoluteDate getDate() {
        return epoch;
    }

    /**
     * Get the covariance matrix.
     * @return the covariance matrix
     */
    public RealMatrix getMatrix() {
        return orbitalCovariance;
    }

    /**
     * Get the covariance orbit type.
     * @return the covariance orbit type
     */
    public OrbitType getOrbitType() {
        return orbitType;
    }

    /**
     * Get the covariance angle type.
     * @return the covariance angle type
     */
    public PositionAngle getPositionAngle() {
        return angleType;
    }

    /**
     * Get the covariance frame.
     * @return the covariance frame (can be null)
     * @see #getLOFType()
     */
    public Frame getFrame() {
        return frame;
    }

    /**
     * Get the covariance LOF type.
     * @return the covariance LOF type (can be null)
     * @see #getFrame()
     */
    public LOFType getLOFType() {
        return lofType;
    }

    /**
     * Get the covariance matrix in another orbit type.
     *
     * @param orbit orbit to which the covariance matrix should correspond
     * @param outOrbitType target orbit type of the state covariance matrix
     * @param outAngleType target position angle type of the state covariance matrix
     * @return a new covariance state, expressed in the target orbit type with the target position angle
     */
    public StateCovariance changeCovarianceType(final Orbit orbit, final OrbitType outOrbitType, final PositionAngle outAngleType) {
        return changeTypeAndCreate(orbit, epoch, frame, orbitType, angleType, outOrbitType, outAngleType, orbitalCovariance);
    }

    /**
     * Get the covariance in a given local orbital frame.
     *
     * @param orbit orbit orbit to which the covariance matrix should correspond
     * @param lofOut output local orbital frame
     * @return a new covariance state, expressed in the output local orbital frame
     */
    public StateCovariance changeCovarianceFrame(final Orbit orbit, final LOFType lofOut) {

        // Verify covariance frame
        if (lofType != null) {

            // Change the covariance local orbital frame
            return changeFrameAndCreate(orbit, epoch, lofType, lofOut, orbitalCovariance);

        } else {

            // Covariance is expressed in celestial body frame
            return changeFrameAndCreate(orbit, epoch, frame, lofOut, orbitalCovariance, orbitType, angleType);

        }

    }

    /**
     * Get the covariance in the output frame.
     *
     * @param orbit orbit orbit to which the covariance matrix should correspond
     * @param frameOut output frame
     * @return a new covariance state, expressed in the output frame
     */
    public StateCovariance changeCovarianceFrame(final Orbit orbit, final Frame frameOut) {

        // Verify covariance frame
        if (lofType != null) {

            // Covariance is expressed in local orbital frame
            return changeFrameAndCreate(orbit, epoch, lofType, frameOut, orbitalCovariance);

        } else {

            // Change covariance frame
            return changeFrameAndCreate(orbit, epoch, frame, frameOut, orbitalCovariance, orbitType, angleType);

        }

    }

    /**
     * Get a time-shifted covariance matrix.
     * <p>
     * The shifting model is a Keplerian one. In other words, the state
     * transition matrix is computed supposing Keplerian motion.
     * <p>
     * Shifting is <em>not</em> intended as a replacement for proper
     * covariance propagation, but should be sufficient for small time
     * shifts or coarse accuracy.
     *
     * @param orbit orbit to which the covariance matrix should correspond
     * @param dt time shift in seconds
     * @return a new covariance state, shifted with respect to the instance
     */
    public StateCovariance shiftedBy(final Orbit orbit, final double dt) {

        // Shifted orbit
        final Orbit shifted = orbit.shiftedBy(dt);

        // Compute STM
        final RealMatrix stm = getStm(orbit, dt);

        // Convert covariance in STM type (i.e., Keplerian elements)
        final StateCovariance inStmType = changeTypeAndCreate(orbit, epoch, frame, orbitType, angleType,
                                                              OrbitType.KEPLERIAN, PositionAngle.MEAN,
                                                              orbitalCovariance);

        // Shift covariance by applying the STM
        final RealMatrix shiftedCov = stm.multiply(inStmType.getMatrix().multiplyTransposed(stm));

        // Restore the initial covariance type
        return changeTypeAndCreate(shifted, shifted.getDate(), frame,
                                   OrbitType.KEPLERIAN, PositionAngle.MEAN,
                                   orbitType, angleType, shiftedCov);

    }

    /**
     * Create a covariance matrix in another orbit type.
     *
     * @param orbit orbit to which the covariance matrix should correspond
     * @param date covariance epoch
     * @param covFrame covariance frame
     * @param inOrbitType initial orbit type of the state covariance matrix
     * @param inAngleType initial position angle type of the state covariance matrix
     * @param outOrbitType target orbit type of the state covariance matrix
     * @param outAngleType target position angle type of the state covariance matrix
     * @param inputCov input covariance
     * @return the covariance expressed in the target orbit type with the target position angle
     */
    private static StateCovariance changeTypeAndCreate(final Orbit orbit, final AbsoluteDate date,
                                                       final Frame covFrame,
                                                       final OrbitType inOrbitType, final PositionAngle inAngleType,
                                                       final OrbitType outOrbitType, final PositionAngle outAngleType,
                                                       final RealMatrix inputCov) {

        // Notations:
        // I: Input orbit type
        // O: Output orbit type
        // C: Cartesian parameters

        // Compute dOutputdCartesian
        final double[][] aOC               = new double[STATE_DIMENSION][STATE_DIMENSION];
        final Orbit      orbitInOutputType = outOrbitType.convertType(orbit);
        orbitInOutputType.getJacobianWrtCartesian(outAngleType, aOC);
        final RealMatrix dOdC = new Array2DRowRealMatrix(aOC, false);

        // Compute dCartesiandInput
        final double[][] aCI              = new double[STATE_DIMENSION][STATE_DIMENSION];
        final Orbit      orbitInInputType = inOrbitType.convertType(orbit);
        orbitInInputType.getJacobianWrtParameters(inAngleType, aCI);
        final RealMatrix dCdI = new Array2DRowRealMatrix(aCI, false);

        // Compute dOutputdInput
        final RealMatrix dOdI = dOdC.multiply(dCdI);

        // Conversion of the state covariance matrix in target orbit type
        final RealMatrix outputCov = dOdI.multiply(inputCov.multiplyTransposed(dOdI));

        // Return the converted covariance
        return new StateCovariance(outputCov, date, covFrame, outOrbitType, outAngleType);

    }

    /**
     * Create a covariance matrix from a {@link LOFType local orbital frame} to another
     * {@link LOFType local orbital frame}.
     * <p>
     * The transformation is based on Equation (20) of "Covariance Transformations for Satellite Flight Dynamics
     * Operations" by David A. Vallado".
     * <p>
     * As this method transforms from and to a {@link LOFType local orbital frame}, it necessarily takes
     * in a covariance matrix expressed in <b>cartesian elements</b> and output a covariance matrix also
     * expressed in <b>cartesian elements</b>.
     *
     * @param orbit orbit to which the covariance matrix should correspond
     * @param date covariance epoch
     * @param lofIn the local orbital frame in which the input covariance matrix is expressed
     * @param lofOut the target local orbital frame
     * @param inputCartesianCov input covariance {@code CARTESIAN})
     * @return the covariance matrix expressed in the target commonly used local orbital frame in cartesian elements
     */
    private static StateCovariance changeFrameAndCreate(final Orbit orbit, final AbsoluteDate date,
                                                        final LOFType lofIn, final LOFType lofOut,
                                                        final RealMatrix inputCartesianCov) {

        // Compute rotation matrix from lofIn to lofOut
        final Rotation rotationFromLofInToLofOut = LOFType.rotationFromLOFInToLOFOut(lofIn, lofOut, orbit.getPVCoordinates());

        // Builds the matrix to perform covariance transformation
        final RealMatrix transformationMatrix = buildTransformationMatrixFromRotation(rotationFromLofInToLofOut);

        // Get the Cartesian covariance matrix converted to frameOut
        final RealMatrix cartesianCovarianceOut = transformationMatrix.multiply(inputCartesianCov.multiplyTransposed(transformationMatrix));

        // Output converted covariance
        return new StateCovariance(cartesianCovarianceOut, date, lofOut);

    }

    /**
     * Convert the covariance matrix from a {@link Frame frame} to a {@link LOFType commonly used local orbital frame}.
     * <p>
     * The transformation is based on Equation (20) of "Covariance Transformations for Satellite Flight Dynamics
     * Operations" by David A. Vallado".
     * <p>
     * As the frame transformation must be performed with the covariance expressed in Cartesian elements, both the orbit
     * and position angle types of the input covariance must be provided.
     * <p>
     * <b>The output covariance matrix will necessarily be expressed in cartesian elements and not converted back to
     * its original expression (if input different from cartesian elements).</b>
     * <p>
     *
     * @param orbit orbit to which the covariance matrix should correspond
     * @param date covariance epoch
     * @param frameIn the frame in which the input covariance matrix is expressed. In case the frame is <b>not</b>
     * pseudo-inertial, the input covariance matrix is expected to be expressed in <b>cartesian elements</b>.
     * @param lofOut the target local orbital frame
     * @param inputCov input covariance
     * @param covOrbitType orbit type of the covariance matrix (used if frameIn is pseudo-inertial)
     * @param covAngleType position angle type of the covariance matrix (used if frameIn is pseudo-inertial) (not used
     * if covOrbitType equals {@code CARTESIAN})
     * @return the covariance matrix expressed in the target local orbital frame in cartesian elements
     * @throws OrekitException if input frame is <b>not</b> pseudo-inertial <b>and</b> the input covariance is
     * <b>not</b> expressed in cartesian elements.
     */
    private static StateCovariance changeFrameAndCreate(final Orbit orbit, final AbsoluteDate date,
                                                        final Frame frameIn, final LOFType lofOut,
                                                        final RealMatrix inputCov,
                                                        final OrbitType covOrbitType, final PositionAngle covAngleType) {

        // Input frame is inertial
        if (frameIn.isPseudoInertial()) {

            // Convert input matrix to Cartesian parameters in input frame
            final RealMatrix cartesianCovarianceIn = changeTypeAndCreate(orbit, date, frameIn, covOrbitType, covAngleType,
                                                                         OrbitType.CARTESIAN, PositionAngle.MEAN,
                                                                         inputCov).getMatrix();

            // Compute rotation matrix from frameIn to lofOut
            final Rotation rotationFromFrameInToLofOut = lofOut.rotationFromInertial(orbit.getPVCoordinates(frameIn));

            // Builds the matrix to perform covariance transformation
            final RealMatrix transformationMatrix = buildTransformationMatrixFromRotation(rotationFromFrameInToLofOut);

            // Get the Cartesian covariance matrix converted to frameOut
            final RealMatrix cartesianCovarianceOut =
                    transformationMatrix.multiply(cartesianCovarianceIn.multiplyTransposed(transformationMatrix));

            // Return converted covariance matrix expressed in cartesian elements
            return new StateCovariance(cartesianCovarianceOut, date, lofOut);

        }

        // Input frame is not inertial so the covariance matrix is expected to be in cartesian elements
        else {
            if (covOrbitType.equals(OrbitType.CARTESIAN)) {
                final Frame orbitInertialFrame = orbit.getFrame();

                // Compute rotation matrix from frameIn to orbit inertial frame
                final RealMatrix cartesianCovarianceInOrbitFrame =
                       changeFrameAndCreate(orbit, date, frameIn, orbitInertialFrame, inputCov,
                                             OrbitType.CARTESIAN, PositionAngle.MEAN).getMatrix();

                // Convert from orbit inertial frame to lofOut
                return changeFrameAndCreate(orbit, date, orbitInertialFrame, lofOut, cartesianCovarianceInOrbitFrame,
                                            OrbitType.CARTESIAN, PositionAngle.MEAN);

            }
            else {
                throw new OrekitException(OrekitMessages.WRONG_ORBIT_PARAMETERS_TYPE, covOrbitType,
                                          OrbitType.CARTESIAN);

            }

        }

    }

    /**
     * Convert the covariance matrix from a {@link LOFType commonly used local orbital frame} to a {@link Frame frame}.
     * <p>
     * The transformation is based on Equation (20) of "Covariance Transformations for Satellite Flight Dynamics
     * Operations" by David A. Vallado".
     * <p>
     * The <u>input</u> covariance matrix is necessarily expressed in <b>cartesian elements</b>.
     * <p>
     * The <u>output</u> covariance matrix will be expressed in <b>cartesian elements</b>.
     *
     * @param orbit orbit to which the covariance matrix should correspond
     * @param date covariance epoch
     * @param lofIn the local orbital frame in which the input covariance matrix is expressed
     * @param frameOut the target frame
     * @param inputCartesianCov input covariance ({@code CARTESIAN})
     * @return the covariance matrix expressed in the target frame in cartesian elements
     */
    private static StateCovariance changeFrameAndCreate(final Orbit orbit, final AbsoluteDate date,
                                                        final LOFType lofIn, final Frame frameOut,
                                                        final RealMatrix inputCartesianCov) {

        // Get the orbit inertial frame
        final Frame orbitInertialFrame = orbit.getFrame();

        if (frameOut.isPseudoInertial()) {

            // Compute rotation matrix from lofIn to frameOut
            final Rotation rotationFromLofInToFrameOut =
                    lofIn.rotationFromInertial(orbit.getPVCoordinates(frameOut)).revert();

            // Builds the matrix to perform covariance transformation
            final RealMatrix transformationMatrix =
                    buildTransformationMatrixFromRotation(rotationFromLofInToFrameOut);

            // Transform covariance
            final RealMatrix transformedCovariance = transformationMatrix.multiply(inputCartesianCov.multiplyTransposed(transformationMatrix));

            // Get the Cartesian covariance matrix converted to frameOut
            return new StateCovariance(transformedCovariance, date, frameOut, OrbitType.CARTESIAN, DEFAULT_POSITION_ANGLE);

        } else {

            // Compute rotation matrix from lofIn to orbit inertial frame
            final Rotation rotationFromLofInToOrbitFrame =
                    lofIn.rotationFromInertial(orbit.getPVCoordinates()).revert();

            // Builds the matrix to perform covariance transformation
            final RealMatrix transformationMatrixFromLofInToOrbitFrame =
                    buildTransformationMatrixFromRotation(rotationFromLofInToOrbitFrame);

            // Get the Cartesian covariance matrix converted to orbit inertial frame
            final RealMatrix cartesianCovarianceInOrbitFrame = transformationMatrixFromLofInToOrbitFrame.multiply(
                    inputCartesianCov.multiplyTransposed(transformationMatrixFromLofInToOrbitFrame));

            // Get the Cartesian covariance matrix converted to frameOut
            return changeFrameAndCreate(orbit, date, orbitInertialFrame, frameOut, cartesianCovarianceInOrbitFrame,
                                        OrbitType.CARTESIAN, PositionAngle.MEAN);
        }

    }

    /**
     * Get the covariance matrix in another frame.
     * <p>
     * The transformation is based on Equation (18) of "Covariance Transformations for Satellite Flight Dynamics
     * Operations" by David A. Vallado".
     * <p>
     * As the frame transformation must be performed with the covariance expressed in Cartesian elements, both the orbit
     * and position angle types of the input covariance must be provided.
     * <p>
     * In case the <u>input</u> frame is <b>not</b> pseudo-inertial, the <u>input</u> covariance matrix is necessarily
     * expressed in <b>cartesian elements</b>.
     * <p>
     * In case the <u>output</u> frame is <b>not</b> pseudo-inertial, the <u>output</u> covariance matrix will be
     * expressed in <b>cartesian elements</b>.
     *
     * @param orbit orbit to which the covariance matrix should correspond
     * @param date covariance epoch
     * @param frameIn the frame in which the input covariance matrix is expressed
     * @param frameOut the target frame
     * @param inputCov input covariance
     * @param covOrbitType orbit type of the covariance matrix (used if frameIn is pseudo-inertial)
     * @param covAngleType position angle type of the covariance matrix (used if frameIn is pseudo-inertial) (<b>not</b>
     * used if covOrbitType equals {@code CARTESIAN})
     * @return the covariance matrix expressed in the target frame
     * @throws OrekitException if input frame is <b>not</b> pseudo-inertial <b>and</b> the input covariance is
     * <b>not</b> expressed in cartesian elements.
     */
    private static StateCovariance changeFrameAndCreate(final Orbit orbit, final AbsoluteDate date,
                                                        final Frame frameIn, final Frame frameOut,
                                                        final RealMatrix inputCov,
                                                        final OrbitType covOrbitType, final PositionAngle covAngleType) {

        // Get the transform from the covariance frame to the output frame
        final Transform inToOut = frameIn.getTransformTo(frameOut, orbit.getDate());

        // Get the Jacobian of the transform
        final double[][] jacobian = new double[STATE_DIMENSION][STATE_DIMENSION];
        inToOut.getJacobian(CartesianDerivativesFilter.USE_PV, jacobian);

        // Matrix to perform the covariance transformation
        final RealMatrix j = new Array2DRowRealMatrix(jacobian, false);

        // Input frame pseudo-inertial
        if (frameIn.isPseudoInertial()) {

            // Convert input matrix to Cartesian parameters in input frame
            final RealMatrix cartesianCovarianceIn = changeTypeAndCreate(orbit, date, frameIn, covOrbitType, covAngleType,
                                                                         OrbitType.CARTESIAN, PositionAngle.MEAN,
                                                                         inputCov).getMatrix();

            // Get the Cartesian covariance matrix converted to frameOut
            final RealMatrix cartesianCovarianceOut = j.multiply(cartesianCovarianceIn.multiplyTransposed(j));

            // Output frame is pseudo-inertial
            if (frameOut.isPseudoInertial()) {

                // Convert output Cartesian matrix to initial orbit type and position angle
                return changeTypeAndCreate(orbit, date, frameOut, OrbitType.CARTESIAN, PositionAngle.MEAN,
                                           covOrbitType, covAngleType, cartesianCovarianceOut);

            } else {

                // Output frame is not pseudo-inertial -> Output cartesian matrix
                return new StateCovariance(cartesianCovarianceOut, date, frameOut, OrbitType.CARTESIAN, DEFAULT_POSITION_ANGLE);

            }

        } else {

            // Input frame is not pseudo-inertial

            // Covariance is expressed in cartesian elements
            if (covOrbitType.equals(OrbitType.CARTESIAN)) {

                // Convert covariance matrix to frameOut
                final RealMatrix covInFrameOut = j.multiply(inputCov.multiplyTransposed(j));

                // Output the Cartesian covariance matrix converted to frameOut
                return new StateCovariance(covInFrameOut, date, frameOut, OrbitType.CARTESIAN, DEFAULT_POSITION_ANGLE);

            }
            // Covariance is not expressed in cartesian elements
            else {
                throw new OrekitException(OrekitMessages.WRONG_ORBIT_PARAMETERS_TYPE, covOrbitType,
                                          OrbitType.CARTESIAN);
            }

        }

    }

    /**
     * Builds the matrix to perform covariance transformation.
     * @param rotation input rotation
     * @return the matrix to perform the covariance transformation
     */
    private static RealMatrix buildTransformationMatrixFromRotation(final Rotation rotation) {

        // Rotation
        final double[][] rotationMatrixData = rotation.getMatrix();

        // Fills in the upper left and lower right blocks with the rotation
        final RealMatrix transformationMatrix = MatrixUtils.createRealMatrix(STATE_DIMENSION, STATE_DIMENSION);
        transformationMatrix.setSubMatrix(rotationMatrixData, 0, 0);
        transformationMatrix.setSubMatrix(rotationMatrixData, 3, 3);

        // Return
        return transformationMatrix;

    }

    /**
     * Get the state transition matrix considering Keplerian contribution only.
     *
     * @param initialOrbit orbit to which the initial covariance matrix should correspond
     * @param dt time difference between the two orbits
     * @return the state transition matrix used to shift the covariance matrix
     */
    private RealMatrix getStm(final Orbit initialOrbit, final double dt) {

        // initialize the STM
        final RealMatrix stm = MatrixUtils.createRealIdentityMatrix(STATE_DIMENSION);

        // State transition matrix using Keplerian contribution only
        final double mu  = initialOrbit.getMu();
        final double sma = initialOrbit.getA();
        final double contribution = -1.5 * dt * FastMath.sqrt(mu / FastMath.pow(sma, 5));
        stm.setEntry(5, 0, contribution);

        // Return
        return stm;

    }

}
