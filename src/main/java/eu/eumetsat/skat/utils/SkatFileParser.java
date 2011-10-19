package eu.eumetsat.skat.utils;

import java.io.IOException;
import java.io.InputStream;

import org.antlr.runtime.ANTLRInputStream;
import org.antlr.runtime.CommonTokenStream;
import org.antlr.runtime.RecognitionException;
import org.apache.commons.math.exception.DimensionMismatchException;
import org.apache.commons.math.exception.util.DummyLocalizable;
import org.apache.commons.math.exception.util.Localizable;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.linear.Array2DRowRealMatrix;
import org.apache.commons.math.linear.RealMatrix;
import org.apache.commons.math.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitMessages;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Predefined;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;

/** Simple parser for key/value files.
 * <p>
 * This file is a slightly modified copy (changed package name)
 * of the similar file present in the Orekit library tutorials,
 * which is distributed under the terms of the Apache Software
 * License V2.
 * </p>
 * @param Key type of the parameter keys
 */
public class SkatFileParser<Key extends Enum<Key>> {

    /** Error message for unknown frame. */
    private static final Localizable UNKNOWN_FRAME =
        new DummyLocalizable("unknown frame {0}");

    /** Error message for not Earth frame. */
    private static final Localizable NOT_EARTH_FRAME =
        new DummyLocalizable("frame {0} is not an Earth frame");

    /** Data root node. */
    private SkatParseTree root;

    /** Current node. */
    private SkatParseTree current;

    /** Simple constructor.
     */
    public SkatFileParser() {
    }

    /** Parse an input file.
     * <p>
     * The input file syntax is a set of key=value lines. Blank lines and lines
     * starting with '#' (after whitespace trimming) are silently ignored. The
     * equal sign may be surrounded by space characters. Keys must correspond to
     * the {@link Key} enumerate constants, given that matching is not
     * case sensitive and that '_' characters may appear as '.' characters in the
     * file. This means that the lines:
     * <pre>
     *   # this is the semi-major axis
     *   orbit.circular.a   = 7231582
     * </pre>
     * are perfectly right and correspond to key {@code Key#ORBIT_CIRCULAR_A} if
     * such a constant exists in the enumerate.
     * </p>
     * @param input input stream
     * @exception IOException if input file cannot be read
     * @exception RecognitionException if a syntax error occurs in the input file
     * @exception IllegalArgumentException if a line cannot be read properly
     */
    public void parseInput(final InputStream input)
        throws IOException, RecognitionException, IllegalArgumentException {

        SkatLexer lexer = new SkatLexer(new ANTLRInputStream(input));
        CommonTokenStream tokensStream = new CommonTokenStream();
        tokensStream.setTokenSource(lexer);
        SkatParser parser = new SkatParser(tokensStream);
        root = (SkatParseTree) parser.data().getTree();
        current = root;
    }

    /** Enter a branch in the data tree.
     * @param key key of the selected branch
     * @exception NoSuchFieldException if the branch does not exist
     */
    public void enterBranch(final Key key) throws NoSuchFieldException {
        current = current.getField(key.name());
    }

    /** Leave a branch in the data tree.
     */
    public void leaveBranch() {
        current = (SkatParseTree) current.getParent();
    }

    /** Check if the current branch contains a parameter.
     * @param key parameter key
     * @return true if the branch contains the requested parameter
     */
    public boolean containsKey(final Key key) {
        return current.containsField(key.name());
    }

    /** Get a raw double value from a parameters map.
     * @param key parameter key
     * @return double value corresponding to the key
     * @exception NoSuchFieldException if key is not in the map
     */
    public double getDouble(final Key key) throws NoSuchFieldException {
        return current.getField(key.name()).getDoubleValue();
    }

    /** Get a raw int value from a parameters map.
     * @param key parameter key
     * @return int value corresponding to the key
     * @exception NoSuchFieldException if key is not in the map
     */
    public int getInt(final Key key) throws NoSuchFieldException {
        return current.getField(key.name()).getIntValue();
    }

    /** Get an angle value from a parameters map.
     * <p>
     * The angle is considered to be in degrees in the file, it will be returned in radians
     * </p>
     * @param key parameter key
     * @return angular value corresponding to the key, in radians
     * @exception NoSuchFieldException if key is not in the map
     */
    public double getAngle(final Key key) throws NoSuchFieldException {
        return FastMath.toRadians(getDouble(key));
    }

    /** Get a date value from a parameters map.
     * <p>
     * The date is considered to be in UTC in the file
     * </p>
     * @param key parameter key
     * @param timeScale time scale in which the date is to be parsed
     * @return date value corresponding to the key
     * @exception NoSuchFieldException if key is not in the map
     */
    public AbsoluteDate getDate(final Key key, TimeScale timeScale) throws NoSuchFieldException {
        return current.getField(key.name()).getDateValue(timeScale);
    }

    /** Get a string value from a parameters map.
     * @param key parameter key
     * @return string value corresponding to the key
     * @exception NoSuchFieldException if key is not in the map
     */
    public String getString(final Key key) throws NoSuchFieldException {
        return current.getField(key.name()).getStringValue();
    }

    /** Get a vector value from a parameters map.
     * @param key parameter key
     * @return vector value corresponding to the key
     * @exception NoSuchFieldException if key is not in the map
     * @exception DimensionMismatchException if array does not have 3 elements
     */
    public Vector3D getVector(final Key key)
        throws NoSuchFieldException {
        final double[] array = current.getField(key.name()).getDoubleArrayValue();
        if (array.length != 3) {
            throw new DimensionMismatchException(array.length, 3);
        }
        return new Vector3D(array[0], array[1], array[2]);
    }

    /** Get a covariance value from a parameters map.
     * @param key parameter key
     * @return covariance value corresponding to the key
     * @exception NoSuchFieldException if key is not in the map
     * @exception DimensionMismatchException if matrix does not have 6x6 elements
     */
    public RealMatrix getCovariance(final Key key)
        throws NoSuchFieldException {

        final double[][] matrix = current.getField(key.name()).getDoubleMatrixValue();

        if (matrix.length != 6) {
            throw new DimensionMismatchException(matrix.length, 6);
        }
        for (double[] row : matrix) {
            if (row.length != 6) {
                throw new DimensionMismatchException(row.length, 6);
            }
        }

        return new Array2DRowRealMatrix(matrix, false);

    }

    /** Get an inertial frame from a parameters map.
     * @param key parameter key
     * @return inertial frame corresponding to the key
     * @exception NoSuchFieldException if key is not in the map
     * @exception OrekitException if frame cannot be built
     */
    public Frame getInertialFrame(final Key key) throws NoSuchFieldException, OrekitException {

        // get the name of the desired frame
        final String frameName = current.getField(key.name()).getStringValue();

        // check the name against predefined frames
        for (Predefined predefined : Predefined.values()) {
            if (frameName.equals(predefined.getName())) {
                if (FramesFactory.getFrame(predefined).isPseudoInertial()) {
                    return FramesFactory.getFrame(predefined);
                } else {
                    throw new OrekitException(OrekitMessages.NON_PSEUDO_INERTIAL_FRAME_NOT_SUITABLE_FOR_DEFINING_ORBITS,
                                              frameName);
                }
            }
        }

        // none of the frames match the name
        throw new OrekitException(UNKNOWN_FRAME, frameName);

    }

    /** Get an Earth frame from a parameters map.
     * <p>
     * We consider Earth frames are the frames with name starting with "ITRF".
     * </p>
     * @param key parameter key
     * @param parameters key/value map containing the parameters
     * @return Earth frame corresponding to the key
     * @exception NoSuchFieldException if key is not in the map
     * @exception OrekitException if frame cannot be built
     */
    public Frame getEarthFrame(final Key key)
            throws NoSuchFieldException, OrekitException {

        // get the name of the desired frame
        final String frameName = current.getField(key.name()).getStringValue();

        // check the name against predefined frames
        for (Predefined predefined : Predefined.values()) {
            if (frameName.equals(predefined.getName())) {
                if (frameName.startsWith("ITRF")) {
                    return FramesFactory.getFrame(predefined);
                } else {
                    throw new OrekitException(NOT_EARTH_FRAME, frameName);
                }
            }
        }

        // none of the frames match the name
        throw new OrekitException(UNKNOWN_FRAME, frameName);

    }

}
