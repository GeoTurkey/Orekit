package eu.eumetsat.skat.utils;

import java.io.IOException;
import java.io.InputStream;

import org.antlr.runtime.ANTLRInputStream;
import org.antlr.runtime.CommonTokenStream;
import org.antlr.runtime.RecognitionException;
import org.antlr.runtime.tree.Tree;
import org.apache.commons.math.exception.DimensionMismatchException;
import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Predefined;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.EquinoctialOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.utils.PVCoordinates;

/** Simple parser for key/value files with embedded structures and arrays.
 */
public class SkatFileParser {

    /** Name of the input for error messages. */
    private String inputName;

    /** Data tree root node. */
    private final Tree root;

    /** Simple constructor.
     * @param inputName name of the input (for error messages)
     * @param input input stream
     * @exception IOException if input file cannot be read
     * @exception RecognitionException if a syntax error occurs in the input file
     * @exception IllegalArgumentException if a line cannot be read properly
     */
    public SkatFileParser(final String inputName, final InputStream input)
        throws IOException, RecognitionException, IllegalArgumentException {

        this.inputName = inputName;

        SkatLexer lexer = new SkatLexer(new ANTLRInputStream(input));
        CommonTokenStream tokensStream = new CommonTokenStream();
        tokensStream.setTokenSource(lexer);
        SkatParser parser = new SkatParser(tokensStream);

        root = (Tree) parser.data().getTree();

    }

    /** Get the root of the parsed data tree.
     * @return root of the parsed data tree
     */
    public Tree getRoot() {
        return root;
    }

    /** Get the name of the input file.
     * @return name of the input file
     */
    public String getInputName() {
        return inputName;
    }

    /** Check if a structure node contains a value for a specified key.
     * @param node structure node
     * @param key field key
     * @return true if the field exists
     * @exception IllegalArgumentException if the node is not a structure node
     */
    public boolean containsKey(final Tree node, final ParameterKey key)
        throws IllegalArgumentException {

        checkType(SkatParser.STRUCT, node);

        for (int i = 0; i < node.getChildCount(); ++i) {
            final Tree child = node.getChild(i);
            checkType(SkatParser.ASSIGNMENT, child);
         // the key is the first child
            if (child.getChild(0).getText().equals(key.getKey())) {
                return true;
            }
        }

        return false;

    }

    /** Get the value associated with a specified key.
     * @param node structure node
     * @param key key to which the value is associated
     * @return field
     * @exception IllegalArgumentException if the node is not a structure node
     * or if the node does not contain a value for the specified key
     */
    public Tree getValue(final Tree node, final ParameterKey key)
        throws IllegalArgumentException {

        checkType(SkatParser.STRUCT, node);

        for (int i = 0; i < node.getChildCount(); ++i) {
            final Tree child = node.getChild(i);
            checkType(SkatParser.ASSIGNMENT, child);
            // the key is the first child and the value is the second child
            if (child.getChild(0).getText().equals(key.getKey())) {
                return child.getChild(1);
            }
        }

        throw SkatException.createIllegalArgumentException(SkatMessages.MISSING_INPUT_DATA,
                                                           node.getLine(), inputName,
                                                           key.getKey());

    }

    /** Get the number of elements in an array.
     * @param node structure node
     * @return number of elements in the array
     * @exception IllegalArgumentException if the node is not an array node
     */
    public int getElementsNumber(final Tree node)
        throws IllegalArgumentException {

        checkType(SkatParser.ARRAY, node);
        return node.getChildCount();

    }

    /** Get an array element.
     * @param i array element index
     * @return array element
     * @exception IllegalArgumentException if the node is not an array node
     * or if the node does not contain a value for the specified key
     */
    public Tree getElement(final Tree node, final int i)
        throws IllegalArgumentException {

        checkType(SkatParser.ARRAY, node);
        if ((i < 0) || (i >= node.getChildCount())) {
            throw SkatException.createIllegalArgumentException(SkatMessages.MISSING_INPUT_DATA,
                                                               node.getLine(), inputName,
                                                               "[" + i + "]");
        }
        return node.getChild(i);

    }

    /** Get a double value.
     * @param node structure containing the parameter
     * @param key parameter key
     * @return double value corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not a number
     */
    public double getDouble(final Tree node, final ParameterKey key) throws IllegalArgumentException {

        // get the node
        final Tree value = getValue(node, key);

        // check its type (both integers and doubles can be parsed as doubles)
        try {
            checkType(SkatParser.INT, value);
        } catch (IllegalArgumentException iae) {
            checkType(SkatParser.DOUBLE, value);
        }

        // parse the value
        return Double.parseDouble(value.getText());

    }

    /** Get an int value.
     * @param node structure containing the parameter
     * @param key parameter key
     * @return int value corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not an integer
     */
    public int getInt(final Tree node, final ParameterKey key) throws IllegalArgumentException {

        // get the node
        final Tree value = getValue(node, key);

        // check its type
        checkType(SkatParser.INT, value);

        // parse the value
        return Integer.parseInt(value.getText());

    }

    /** Get a boolean value.
     * @param node structure containing the parameter
     * @param key parameter key
     * @return boolean value corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not a boolean
     */
    public boolean getBoolean(final Tree node, final ParameterKey key) throws IllegalArgumentException {

        // get the node
        final Tree value = getValue(node, key);

        // check its type
        checkType(SkatParser.BOOLEAN, value);

        // parse the value
        return Boolean.parseBoolean(value.getText());

    }

    /** Get an identifier value.
     * @param node structure containing the parameter
     * @param key parameter key
     * @return identifier value corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not a string
     */
    public String getIdentifier(final Tree node, final ParameterKey key) throws IllegalArgumentException {

        // get the node
        final Tree value = getValue(node, key);

        // check its type
        checkType(SkatParser.IDENTIFIER, value);

        // parse the value
        return value.getText();

    }

    /** Get an identifier value.
     * @param node array containing the parameter
     * @param index index of the identifier in the array
     * @return identifier value corresponding to the index
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not a string
     */
    public String getIdentifier(final Tree node, final int index) throws IllegalArgumentException {

        // get the node
        final Tree value = getElement(node, index);

        // check its type
        checkType(SkatParser.IDENTIFIER, value);

        // parse the value
        return value.getText();

    }

    /** Get a string value.
     * @param node structure containing the parameter
     * @param key parameter key
     * @return string value corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not a string
     */
    public String getString(final Tree node, final ParameterKey key) throws IllegalArgumentException {

        // get the node
        final Tree value = getValue(node, key);

        // check its type
        checkType(SkatParser.STRING, value);

        // parse the value
        return value.getText();

    }

    /** Get an angle value.
     * <p>
     * The angle is considered to be in degrees in the file, it will be returned in radians
     * </p>
     * @param node structure containing the parameter
     * @param key parameter key
     * @return angle value corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not a number
     */
    public double getAngle(final Tree node, final ParameterKey key) throws IllegalArgumentException {
        return FastMath.toRadians(getDouble(node, key));
    }

    /** Get a date value.
     * @param node structure containing the parameter
     * @param key parameter key
     * @param timeScale time scale used to parse the date
     * @return date value corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not a date
     */
    public AbsoluteDate getDate(final Tree node, final ParameterKey key, final TimeScale timeScale)
        throws IllegalArgumentException {

        // get the node
        final Tree value = getValue(node, key);

        // check its type
        checkType(SkatParser.DATE, value);

        // parse the value
        return new AbsoluteDate(value.getText(), timeScale);

    }

    /** Get a position angle type.
     * @param node structure containing the parameter
     * @param key parameter key
     * @return position angle type corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not a position angle
     */
    public PositionAngle getPositionAngle(final Tree node, final ParameterKey key)
        throws IllegalArgumentException {

        // get the node
        final Tree value = getValue(node, key);

        // check its type
        checkType(SkatParser.IDENTIFIER, value);

        // parse the value
        return PositionAngle.valueOf(value.getText());

    }

    /** Get a vector value.
     * @param node structure containing the parameter
     * @param key parameter key
     * @return vector value corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not an array
     * @exception DimensionMismatchException if array does not have 3 elements
     */
    public Vector3D getVector(final Tree node, final ParameterKey key)
        throws IllegalArgumentException {

        // get the node
        final Tree value = getValue(node, key);

        // check its type
        checkType(SkatParser.ARRAY, value);

        // parse the value
        if (getElementsNumber(value) != 3) {
            throw new DimensionMismatchException(getElementsNumber(value), 3);
        }
        return new Vector3D(Double.parseDouble(getElement(value, 0).getText()),
                            Double.parseDouble(getElement(value, 1).getText()),
                            Double.parseDouble(getElement(value, 2).getText()));

    }

    /** Get a double[] value.
     * @param node structure containing the parameter
     * @param key parameter key
     * @return double[] value corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not a matrix
     */
    public double[] getDoubleArray1(final Tree node, final ParameterKey key)
        throws IllegalArgumentException {


        // get the node
        final Tree value = getValue(node, key);

        // check types and get dimensions
        checkType(SkatParser.ARRAY, value);

        // parse the value
        final double[] array = new double[getElementsNumber(value)];
        for (int i = 0; i < array.length; ++i) {
            array[i] = Double.parseDouble(getElement(value, i).getText());
        }
        return array;

    }

    /** Get a double[][] value.
     * @param node structure containing the parameter
     * @param key parameter key
     * @return double[][] value corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not a matrix
     */
    public double[][] getDoubleArray2(final Tree node, final ParameterKey key)
        throws IllegalArgumentException {


        // get the node
        final Tree value = getValue(node, key);

        // check types and get dimensions
        checkType(SkatParser.ARRAY, value);
        final int rows = getElementsNumber(value);
        int columns = 0;
        for (int i = 0; i < rows; ++i) {
            final Tree row = getElement(value, i);
            checkType(SkatParser.ARRAY, row);
            columns = getElementsNumber(row);
        }

        // parse the value
        final double[][] array = new double[rows][columns];
        for (int i = 0; i < array.length; ++i) {
            final Tree row = getElement(value, i);
            for (int j = 0; j < array[i].length; ++j) {
                array[i][j] = Double.parseDouble(getElement(row, j).getText());
            }
        }
        return array;

    }

    /** Get an inertial frame.
     * @param node structure containing the parameter
     * @param key parameter key
     * @return inertial frame corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not a known inertial frame
     * @exception OrekitException if frame cannot be built
     */
    public Frame getInertialFrame(final Tree node, final ParameterKey key)
        throws IllegalArgumentException, OrekitException {

        // get the name of the desired frame
        final String frameName = getString(node, key);

        // check the name against predefined frames
        for (Predefined predefined : Predefined.values()) {
            if (frameName.equals(predefined.getName())) {
                if (FramesFactory.getFrame(predefined).isPseudoInertial()) {
                    return FramesFactory.getFrame(predefined);
                } else {
                    throw new OrekitException(SkatMessages.NOT_INERTIAL_FRAME, frameName);
                }
            }
        }

        // none of the frames match the name
        throw new OrekitException(SkatMessages.UNKNOWN_FRAME, frameName);

    }

    /** Get an Earth frame.
     * <p>
     * We consider Earth frames are the frames with name starting
     * with either "ITRF" or "GTOD".
     * </p>
     * @param node structure containing the parameter
     * @param key parameter key
     * @return Earth frame corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not a known Earth frame
     * @exception OrekitException if frame cannot be built
     */
    public Frame getEarthFrame(final Tree node, final ParameterKey key)
            throws IllegalArgumentException, OrekitException {

        // get the name of the desired frame
        final String frameName = getString(node, key);

        // check the name against predefined frames
        for (Predefined predefined : Predefined.values()) {
            if (frameName.equals(predefined.getName())) {
                if (frameName.startsWith("ITRF") || frameName.startsWith("GTOD")) {
                    return FramesFactory.getFrame(predefined);
                } else {
                    throw new OrekitException(SkatMessages.NOT_EARTH_FRAME, frameName);
                }
            }
        }

        // none of the frames match the name
        throw new OrekitException(SkatMessages.UNKNOWN_FRAME, frameName);

    }

    /** Get an orbit.
     * <p>
     * Orbits may be defined in Cartesian, Keplerian, circular or equinoctial elements.
     * </p>
     * @param node structure containing the parameter
     * @param inertialFrame inertial frame in which orbit is defined
     * @param timeScale time scale in which orbit is defined
     * @param mu gravity coefficient to use
     * @return orbit corresponding to the key
     * @exception IllegalArgumentException if node does not contains the key
     * or it is not an orbit
     * @exception OrekitException if orbit cannot be built
     */
    public Orbit getOrbit(final Tree node, final Frame inertialFrame,
                          final TimeScale timeScale, final double mu)
        throws IllegalArgumentException, OrekitException {

        final String type = getIdentifier(node, ParameterKey.ORBIT_TYPE);
        switch (OrbitType.valueOf(type)) {

        case CARTESIAN :
            return new CartesianOrbit(new PVCoordinates(getVector(node, ParameterKey.ORBIT_CARTESIAN_POSITION),
                                                        getVector(node, ParameterKey.ORBIT_CARTESIAN_VELOCITY)),
                                      inertialFrame,
                                      getDate(node, ParameterKey.ORBIT_CARTESIAN_DATE, timeScale),
                                      mu);
        case KEPLERIAN :
            return new KeplerianOrbit(getDouble(node, ParameterKey.ORBIT_KEPLERIAN_A),
                                      getDouble(node, ParameterKey.ORBIT_KEPLERIAN_E),
                                      getAngle(node,  ParameterKey.ORBIT_KEPLERIAN_I),
                                      getAngle(node,  ParameterKey.ORBIT_KEPLERIAN_PA),
                                      getAngle(node,  ParameterKey.ORBIT_KEPLERIAN_RAAN),
                                      getAngle(node,  ParameterKey.ORBIT_KEPLERIAN_ANOMALY),
                                      getPositionAngle(node, ParameterKey.ANGLE_TYPE),
                                      inertialFrame,
                                      getDate(node, ParameterKey.ORBIT_KEPLERIAN_DATE, timeScale),
                                      mu);
        case CIRCULAR :
            return new CircularOrbit(getDouble(node, ParameterKey.ORBIT_CIRCULAR_A),
                                     getDouble(node, ParameterKey.ORBIT_CIRCULAR_EX),
                                     getDouble(node, ParameterKey.ORBIT_CIRCULAR_EY),
                                     getAngle(node,  ParameterKey.ORBIT_CIRCULAR_I),
                                     getAngle(node,  ParameterKey.ORBIT_CIRCULAR_RAAN),
                                     getAngle(node,  ParameterKey.ORBIT_CIRCULAR_LATITUDE_ARGUMENT),
                                     getPositionAngle(node, ParameterKey.ANGLE_TYPE),
                                     inertialFrame,
                                     getDate(node, ParameterKey.ORBIT_CIRCULAR_DATE, timeScale),
                                     mu);
        case EQUINOCTIAL :
            return new EquinoctialOrbit(getDouble(node, ParameterKey.ORBIT_EQUINOCTIAL_A),
                                        getDouble(node, ParameterKey.ORBIT_EQUINOCTIAL_EX),
                                        getDouble(node, ParameterKey.ORBIT_EQUINOCTIAL_EY),
                                        getDouble(node, ParameterKey.ORBIT_EQUINOCTIAL_HX),
                                        getDouble(node, ParameterKey.ORBIT_EQUINOCTIAL_HY),
                                        getAngle(node, ParameterKey.ORBIT_EQUINOCTIAL_LONGITUDE_ARGUMENT),
                                        getPositionAngle(node, ParameterKey.ANGLE_TYPE),
                                        inertialFrame,
                                        getDate(node, ParameterKey.ORBIT_EQUINOCTIAL_DATE, timeScale),
                                        mu);
        default :
            // this should never happen
            throw SkatException.createInternalError(null);
        }

    }

    /** Check a parser tree node type.
     * @param expected expected type
     * @param node tree node to check
     * @exception IllegalArgumentException if the node has not the expected type
     */
    private void checkType(final int expected, final Tree node)
        throws IllegalArgumentException {
        if (node.getType() != expected) {
            throw SkatException.createIllegalArgumentException(SkatMessages.WRONG_TYPE,
                                                               node.getLine(), inputName,
                                                               SkatParser.tokenNames[expected],
                                                               SkatParser.tokenNames[node.getType()]);
        }
    }

}
