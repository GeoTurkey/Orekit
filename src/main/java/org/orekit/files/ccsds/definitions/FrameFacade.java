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
package org.orekit.files.ccsds.definitions;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.orekit.data.DataContext;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitIllegalArgumentException;
import org.orekit.errors.OrekitMessages;
import org.orekit.frames.Frame;
import org.orekit.frames.LOFType;
import org.orekit.frames.Transform;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinatesProvider;

/** Facade in front of several frames types in CCSDS messages.
 * @author Luc Maisonobe
 * @author Vincent Cucchietti
 * @since 11.0
 */
public class FrameFacade {

    /** Reference to node in Orekit frames tree. */
    private final Frame frame;

    /** Reference to celestial body centered frame. */
    private final CelestialBodyFrame celestialBodyFrame;

    /** Reference to orbit-relative frame. */
    private final OrbitRelativeFrame orbitRelativeFrame;

    /** Reference to spacecraft body frame. */
    private final SpacecraftBodyFrame spacecraftBodyFrame;

    /** Name of the frame. */
    private final String name;

    /** Simple constructor.
     * <p>
     * At most one of {@code celestialBodyFrame}, {@code orbitRelativeFrame}
     * or {@code spacecraftBodyFrame} may be non null. They may all be null
     * if frame is unknown, in which case only the name will be available.
     * </p>
     * @param frame reference to node in Orekit frames tree (may be null)
     * @param celestialBodyFrame reference to celestial body centered frame (may be null)
     * @param orbitRelativeFrame reference to orbit-relative frame (may be null)
     * @param spacecraftBodyFrame reference to spacecraft body frame (may be null)
     * @param name name of the frame
     */
    public FrameFacade(final Frame frame,
                       final CelestialBodyFrame celestialBodyFrame,
                       final OrbitRelativeFrame orbitRelativeFrame,
                       final SpacecraftBodyFrame spacecraftBodyFrame,
                       final String name) {
        this.frame               = frame;
        this.celestialBodyFrame  = celestialBodyFrame;
        this.orbitRelativeFrame  = orbitRelativeFrame;
        this.spacecraftBodyFrame = spacecraftBodyFrame;
        this.name                = name;
    }

    /**
     * Get the associated frame tree node.
     * @return associated frame tree node, or null if none exists
     */
    public Frame asFrame() {
        return frame;
    }

    /** Get the associated {@link CelestialBodyFrame celestial body frame}.
     * @return associated celestial body frame, or null if frame is
     * associated to a {@link #asOrbitRelativeFrame() orbit},
     * a {@link #asSpacecraftBodyFrame spacecraft} or is not supported
     */
    public CelestialBodyFrame asCelestialBodyFrame() {
        return celestialBodyFrame;
    }

    /** Get the associated {@link OrbitRelativeFrame orbit relative frame}.
     * @return associated orbit relative frame, or null if frame is
     * associated to a {@link #asCelestialBodyFrame() celestial body},
     * a {@link #asSpacecraftBodyFrame spacecraft} or is not supported
     */
    public OrbitRelativeFrame asOrbitRelativeFrame() {
        return orbitRelativeFrame;
    }

    /** Get the associated {@link SpacecraftBodyFrame spacecraft body frame}.
     * @return associated spacecraft body frame, or null if frame is
     * associated to a {@link #asCelestialBodyFrame() celestial body},
     * an {@link #asOrbitRelativeFrame orbit} or is not supported
     */
    public SpacecraftBodyFrame asSpacecraftBodyFrame() {
        return spacecraftBodyFrame;
    }

    /** Get the CCSDS name for the frame.
     * @return CCSDS name
     */
    public String getName() {
        return name;
    }

    /** Map an Orekit frame to a CCSDS frame facade.
     * @param frame a reference frame.
     * @return the CCSDS frame corresponding to the Orekit frame
     */
    public static FrameFacade map(final Frame frame) {
        final CelestialBodyFrame cbf = CelestialBodyFrame.map(frame);
        return new FrameFacade(frame, cbf, null, null, cbf.getName());
    }

    /** Simple constructor.
     * @param name name of the frame
     * @param conventions IERS conventions to use
     * @param simpleEOP if true, tidal effects are ignored when interpolating EOP
     * @param dataContext to use when creating the frame
     * @param allowCelestial if true, {@link CelestialBodyFrame} are allowed
     * @param allowOrbit if true, {@link OrbitRelativeFrame} are allowed
     * @param allowSpacecraft if true, {@link SpacecraftBodyFrame} are allowed
     * @return frame facade corresponding to the CCSDS name
     */
    public static FrameFacade parse(final String name,
                                    final IERSConventions conventions,
                                    final boolean simpleEOP,
                                    final DataContext dataContext,
                                    final boolean allowCelestial,
                                    final boolean allowOrbit,
                                    final boolean allowSpacecraft) {
        try {
            final CelestialBodyFrame cbf = CelestialBodyFrame.parse(name);
            if (allowCelestial) {
                return new FrameFacade(cbf.getFrame(conventions, simpleEOP, dataContext),
                                       cbf, null, null, cbf.getName());
            }
        } catch (IllegalArgumentException iaeC) {
            try {
                final OrbitRelativeFrame orf = OrbitRelativeFrame.valueOf(name.replace(' ', '_'));
                if (allowOrbit) {
                    return new FrameFacade(null, null, orf, null, orf.name());
                }
            } catch (IllegalArgumentException iaeO) {
                try {
                    final SpacecraftBodyFrame sbf = SpacecraftBodyFrame.parse(name.replace(' ', '_'));
                    if (allowSpacecraft) {
                        return new FrameFacade(null, null, null, sbf, sbf.toString());
                    }
                } catch (OrekitException | IllegalArgumentException e) {
                    // nothing to do here, use fallback below
                }
            }
        }

        // we don't know any frame with this name, just store the name itself
        return new FrameFacade(null, null, null, null, name);

    }

    /**
     * Computes the transform from one {@link FrameFacade CCCSDS frame} to the other.
     * <p>
     * Note that the pivot frame provided <b>must be inertial</b> and <b>coherent</b> to the frames you are working
     * with.
     * </p>
     *
     * @param frameIn the input {@link FrameFacade CCSDS frame} to convert from
     * @param frameOut the output {@link FrameFacade CCSDS frame}  to convert to
     * @param pivotFrame <b>Inertial</b> frame used as a pivot to create the transform
     * @param date the date for the transform
     * @param pv the PV coordinates provider (required when one of the frames is a LOF)
     * @return the transform from one {@link FrameFacade CCCSDS frame} to the other
     */
    public static Transform getTransform(final FrameFacade frameIn, final FrameFacade frameOut, final Frame pivotFrame,
                                         final AbsoluteDate date, final PVCoordinatesProvider pv) {

        // Gets transform according to the types of the input frames
        if (frameIn.asFrame() != null) {
            return getTransform(frameIn.asFrame(), frameOut, date, pv);

        }
        else if (frameIn.asOrbitRelativeFrame() != null) {
            return getTransform(frameIn.asOrbitRelativeFrame(), frameOut, pivotFrame, date, pv);
        }

        // Transform cannot be gotten from these 2 frames
        throw new OrekitIllegalArgumentException(OrekitMessages.INVALID_TRANSFORM, frameIn.getName(),
                                                 frameOut.getName());

    }

    /**
     * Computes the transform from an {@link Frame Orekit frame} to a  {@link FrameFacade CCSDS frame}.
     *
     * @param frameIn the input {@link Frame Orekit frame} to convert from
     * @param frameOut the output {@link FrameFacade CCSDS frame} to convert to
     * @param date the date for the transform
     * @param pv the PV coordinates provider (required when one of the frames is a
     * {@link org.orekit.frames.LocalOrbitalFrame local orbital frame})
     * @return the transform from an {@link Frame Orekit frame} to a  {@link FrameFacade CCSDS frame}
     */
    public static Transform getTransform(final Frame frameIn, final FrameFacade frameOut,
                                         final AbsoluteDate date, final PVCoordinatesProvider pv) {

        // Gets transform according to the types of the input frames
        if (frameOut.asFrame() != null) {
            return frameIn.getTransformTo(frameOut.asFrame(), date);

        }
        else if (frameOut.asOrbitRelativeFrame() != null) {
            final LOFType lofOut = frameOut.asOrbitRelativeFrame().getLofType();

            if (lofOut != null) {
                return lofOut.transformFromInertial(date, pv.getPVCoordinates(date, frameIn));
            }
        }

        // Transform cannot be gotten from these 2 frames
        throw new OrekitIllegalArgumentException(OrekitMessages.INVALID_TRANSFORM, frameIn.getName(),
                                                 frameOut.getName());
    }

    /**
     * Computes the transform from an {@link OrbitRelativeFrame Orekit orbit relative frame} to a
     * {@link FrameFacade CCSDS frame}.
     * <p>
     * Note that the pivot frame provided <b>must be inertial</b> and <b>coherent</b> to the frames you are working
     * with.
     * </p>
     *
     * @param frameIn the input {@link OrbitRelativeFrame Orekit orbit relative frame} to convert from
     * @param frameOut the output {@link FrameFacade CCSDS frame} to convert to
     * @param pivotFrame <b>Inertial</b> frame used as a pivot to create the transform
     * @param date the date for the transform
     * @param pv the PV coordinates provider (required when one of the frames is a LOF)
     * @return the transform from an {@link OrbitRelativeFrame Orekit orbit relative frame} to a
     * {@link FrameFacade CCSDS frame}
     * @throws OrekitIllegalArgumentException if one of the local orbital frame is undefined or if the input pivotFrame
     * is not inertial
     */
    public static Transform getTransform(final OrbitRelativeFrame frameIn, final FrameFacade frameOut,
                                         final Frame pivotFrame,
                                         final AbsoluteDate date, final PVCoordinatesProvider pv) {

        if (pivotFrame.isPseudoInertial()) {

            final LOFType lofIn = frameIn.getLofType();

            if (lofIn != null) {
                if (frameOut.asFrame() != null) {
                    return lofIn.transformFromInertial(date, pv.getPVCoordinates(date, frameOut.asFrame()))
                            .getInverse();

                }
                else if (frameOut.asOrbitRelativeFrame() != null) {

                    final LOFType lofOut = frameOut.asOrbitRelativeFrame().getLofType();

                    if (lofOut != null) {

                        // First rotation from input local orbital frame to inertial pivot
                        final Rotation first =
                                lofIn.rotationFromInertial(pv.getPVCoordinates(date, pivotFrame)).revert();

                        // Second rotation from inertial pivot to output local orbital frame
                        final Rotation second = lofOut.rotationFromInertial(pv.getPVCoordinates(date, pivotFrame));

                        // Composed rotation
                        final Rotation lofInToLofOut = second.applyTo(first);

                        // Returns the composed transform
                        return new Transform(date, lofInToLofOut);
                    }
                }
            }
            // Transform cannot be gotten from these 2 frames
            throw new OrekitIllegalArgumentException(OrekitMessages.INVALID_TRANSFORM, "undefined relative orbit frame",
                                                     frameOut.getName());
        }
        else {
            // Input pivotFrame is not inertial, an exception is thrown
            throw new OrekitIllegalArgumentException(OrekitMessages.NON_PSEUDO_INERTIAL_FRAME, pivotFrame.getName());
        }
    }
}
