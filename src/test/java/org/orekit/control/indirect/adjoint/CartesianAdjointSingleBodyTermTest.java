package org.orekit.control.indirect.adjoint;

import org.hipparchus.CalculusFieldElement;
import org.hipparchus.analysis.differentiation.Gradient;
import org.hipparchus.analysis.differentiation.GradientField;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.Binary64;
import org.hipparchus.util.Binary64Field;
import org.hipparchus.util.MathArrays;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;
import org.orekit.Utils;
import org.orekit.bodies.CelestialBody;
import org.orekit.forces.gravity.SingleBodyAbsoluteAttraction;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.propagation.FieldSpacecraftState;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.DateComponents;
import org.orekit.time.FieldAbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.TimeStampedFieldPVCoordinates;

class CartesianAdjointSingleBodyTermTest {

    @BeforeEach
    public void setUp() {
        Utils.setDataRoot("regular-data");
    }

    @Test
    void testGetters() {
        // GIVEN
        final CelestialBody body = getCelestialBody(Vector3D.MINUS_I);
        final Frame expectedFrame = FramesFactory.getGCRF();
        final CartesianAdjointSingleBodyTerm singleBodyTerm = new CartesianAdjointSingleBodyTerm(body.getGM(), body,
                expectedFrame);
        // WHEN
        final double mu = singleBodyTerm.getMu();
        final Frame actualFrame = singleBodyTerm.getPropagationFrame();
        // THEN
        Assertions.assertEquals(body.getGM(), mu);
        Assertions.assertEquals(expectedFrame, actualFrame);
    }

    @Test
    void testGetVelocityAdjointContribution() {
        // GIVEN
        final CelestialBody body = getCelestialBody(Vector3D.MINUS_I);
        final CartesianAdjointSingleBodyTerm singleBodyTerm = new CartesianAdjointSingleBodyTerm(body.getGM(), body,
                FramesFactory.getGCRF());
        final double[] adjoint = new double[6];
        final double[] state = new double[6];
        for (int i = 0; i < adjoint.length; i++) {
            state[i] = -i+1;
            adjoint[i] = i;
        }
        final AbsoluteDate date = new AbsoluteDate(new DateComponents(2015, 1, 1), TimeScalesFactory.getUTC());
        // WHEN
        final double[] contribution = singleBodyTerm.getVelocityAdjointContribution(date, state, adjoint);
        // THEN
        final SingleBodyAbsoluteAttraction singleBodyAbsoluteAttraction = new SingleBodyAbsoluteAttraction(body);
        final int dimension = 3;
        final GradientField field = GradientField.getField(dimension);
        final FieldSpacecraftState<Gradient> mockedState = Mockito.mock(FieldSpacecraftState.class);
        final FieldVector3D<Gradient> fieldPosition = new FieldVector3D<>(
                Gradient.variable(dimension, 0, state[0]),
                Gradient.variable(dimension, 1, state[1]),
                Gradient.variable(dimension, 2, state[2]));
        Mockito.when(mockedState.getDate()).thenReturn(new FieldAbsoluteDate<>(field, date));
        Mockito.when(mockedState.getPosition()).thenReturn(fieldPosition);
        final Gradient[] fieldMu = MathArrays.buildArray(field, 1);
        fieldMu[0] = Gradient.constant(dimension, singleBodyTerm.getMu());
        final FieldVector3D<Gradient> acceleration = singleBodyAbsoluteAttraction.acceleration(mockedState, fieldMu);
        final double tolerance = 1e-12;
        Assertions.assertEquals(-contribution[0], acceleration.getX().getGradient()[0] * adjoint[3]
                + acceleration.getX().getGradient()[1] * adjoint[4] + acceleration.getX().getGradient()[2] * adjoint[5], tolerance);
        Assertions.assertEquals(-contribution[1], acceleration.getY().getGradient()[0] * adjoint[3]
                + acceleration.getY().getGradient()[1] * adjoint[4] + acceleration.getY().getGradient()[2] * adjoint[5], tolerance);
        Assertions.assertEquals(-contribution[2], acceleration.getZ().getGradient()[0] * adjoint[3]
                + acceleration.getZ().getGradient()[1] * adjoint[4] + acceleration.getZ().getGradient()[2] * adjoint[5], tolerance);
    }


    @Test
    void testGetVelocityAdjointContributionField() {
        // GIVEN
        final CelestialBody celestialBody = getCelestialBody(Vector3D.ZERO);
        final CartesianAdjointSingleBodyTerm cartesianAdjointSingleBodyTerm = new CartesianAdjointSingleBodyTerm(celestialBody.getGM(),
                celestialBody, celestialBody.getInertiallyOrientedFrame());
        final Binary64Field field = Binary64Field.getInstance();
        final Binary64[] fieldAdjoint = MathArrays.buildArray(field, 6);
        final Binary64[] fieldState = MathArrays.buildArray(field, 6);
        for (int i = 0; i < fieldAdjoint.length; i++) {
            fieldState[i] = field.getZero().newInstance(-i+1);
            fieldAdjoint[i] = field.getZero().newInstance(i);
        }
        final FieldAbsoluteDate<Binary64> fieldDate = FieldAbsoluteDate.getArbitraryEpoch(field);
        // WHEN
        final Binary64[] fieldContribution = cartesianAdjointSingleBodyTerm.getVelocityAdjointContribution(fieldDate, fieldState, fieldAdjoint);
        // THEN
        final CartesianAdjointKeplerianTerm keplerianTerm = new CartesianAdjointKeplerianTerm(celestialBody.getGM());
        final Binary64[] contribution = keplerianTerm.getVelocityAdjointContribution(fieldDate, fieldState, fieldAdjoint);
        for (int i = 0; i < contribution.length; i++) {
            Assertions.assertEquals(fieldContribution[i], contribution[i]);
        }
    }

    private static CelestialBody getCelestialBody(final Vector3D position) {
        return new CelestialBody() {
            @Override
            public Frame getInertiallyOrientedFrame() {
                return FramesFactory.getGCRF();
            }

            @Override
            public Frame getBodyOrientedFrame() {
                return null;
            }

            @Override
            public String getName() {
                return "";
            }

            @Override
            public double getGM() {
                return 1.;
            }

            @Override
            public Vector3D getPosition(AbsoluteDate date, Frame frame) {
                return position;
            }

            @Override
            public <T extends CalculusFieldElement<T>> FieldVector3D<T> getPosition(FieldAbsoluteDate<T> date, Frame frame) {
                return new FieldVector3D<>(date.getField(), getPosition(date.toAbsoluteDate(), frame));
            }

            @Override
            public <T extends CalculusFieldElement<T>> TimeStampedFieldPVCoordinates<T> getPVCoordinates(FieldAbsoluteDate<T> date, Frame frame) {
                return null;
            }
        };
    }

}
