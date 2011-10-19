package eu.eumetsat.skat.utils;

import java.util.List;

import org.antlr.runtime.CommonToken;
import org.antlr.runtime.tree.CommonTree;
import org.antlr.runtime.tree.Tree;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;


public class SkatParseTree extends CommonTree {

    /** Enumeration for data typing. */
    public enum SkatType {

        /** Constant for boolean data. */
        BOOLEAN,

        /** Constant for integer data. */
        INT,

        /** Constant for double data. */
        DOUBLE,

        /** Constant for date data. */
        DATE,

        /** Constant for string data. */
        STRING,

        /** Constant for structure data. */
        STRUCTURE,

        /** Constant for array data. */
        ARRAY

    }

    /** Name of the node. */
    private final String name;

    /** Type of the data node. */
    private final SkatType type;

    /** Simple constructor.
     * @param tokenType Skat grammar token type
     * @param name name of the node (may be null)
     * @param type type of the data node
     */
    protected SkatParseTree(final int tokenType, final String name, final SkatType type) {
        super(new CommonToken(tokenType));
        this.name    = name;
        this.type    = type;
    }

    /** Simple constructor.
     * @param tokenType Skat grammar token type
     * @param name name of the node (may be null)
     * @param type type of the data node
     * @param subNodes sub-nodes for structure and arrays
     */
    protected SkatParseTree(final int tokenType, final String name, final SkatType type, final List<?> subNodes) {
        super(new CommonToken(tokenType));
        this.name    = name;
        this.type    = type;
        for (Object sub : subNodes) {
            addChild((Tree) sub);
        }
    }

    /** {@inheritDoc} */
    public SkatParseTree dupNode() {
        return new SkatParseTree(getType(), name, type);
    }

    /** Get the name of the data node.
     * @return name of the data node
     */
    public String getName() {
        return name;
    }

    /** Get the type of the data node.
     * @return type of the data node
     */
    public SkatType getSkatType() {
        return type;
    }

    /** Get the boolean value.
     * @return boolean value of the node
     * @exception IllegalStateException if the node is not a {@link SkatType#BOOLEAN}
     */
    public boolean getBooleanValue()
        throws IllegalStateException {
        if (type != SkatType.BOOLEAN) {
            throw SkatException.createIllegalStateException(SkatMessages.WRONG_TYPE,
                                                            getLine(), SkatType.BOOLEAN, type);
        }
        return Boolean.parseBoolean(getText());
    }

    /** Get the int value.
     * @return int value of the node
     * @exception IllegalStateException if the node is not a {@link SkatType#INT}
     */
    public int getIntValue()
        throws IllegalStateException {
        if (type != SkatType.INT) {
            throw SkatException.createIllegalStateException(SkatMessages.WRONG_TYPE,
                                                            getLine(), SkatType.INT, type);
        }
        return Integer.parseInt(getText());
    }

    /** Get the double value.
     * @return double value of the node
     * @exception IllegalStateException if the node is not a {@link SkatType#DOUBLE}
     */
    public double getDoubleValue()
        throws IllegalStateException {
        if ((type != SkatType.DOUBLE) && (type != SkatType.INT)) {
            throw SkatException.createIllegalStateException(SkatMessages.WRONG_TYPE,
                                                            getLine(), SkatType.DOUBLE, type);
        }
        return Double.parseDouble(getText());
    }

    /** Get the double array value.
     * @return double array value of the node
     * @exception IllegalStateException if the node is not an {@link SkatType#ARRAY}
     * of {@link SkatType#DOUBLE} elements
     */
    public double[] getDoubleArrayValue()
        throws IllegalStateException {
        final double[] array = new double[getElementsNumber()];
        for (int i = 0; i < array.length; ++i) {
            array[i] = getElement(i).getDoubleValue();
        }
        return array;
    }

    /** Get the double matrix value.
     * @return double matrix value of the node
     * @exception IllegalStateException if the node is not an {@link SkatType#ARRAY}
     * of {@link SkatType#ARRAY} of {@link SkatType#DOUBLE} elements
     */
    public double[][] getDoubleMatrixValue()
        throws IllegalStateException {
        final double[][] matrix = new double[getElementsNumber()][];
        for (int i = 0; i < matrix.length; ++i) {
            final SkatParseTree row = getElement(i);
             matrix[i] = new double[row.getElementsNumber()];
            for (int j = 0; j < matrix[i].length; ++j) {
                matrix[i][j] = row.getElement(i).getDoubleValue();
            }
        }
        return matrix;
    }

    /** Get the date value.
     * @param timeScale time scale to use for parsing the date
     * @return date value of the node
     * @exception IllegalStateException if the node is not a {@link SkatType#DATE}
     */
    public AbsoluteDate getDateValue(final TimeScale timeScale)
        throws IllegalStateException {
        if (type != SkatType.DATE) {
            throw SkatException.createIllegalStateException(SkatMessages.WRONG_TYPE,
                                                            getLine(), SkatType.DATE, type);
        }
        return new AbsoluteDate(getText(), timeScale);
    }

    /** Get the string value.
     * @return string value of the node
     * @exception IllegalStateException if the node is not a {@link SkatType#STRING}
     */
    public String getStringValue()
        throws IllegalStateException {
        if (type != SkatType.STRING) {
            throw SkatException.createIllegalStateException(SkatMessages.WRONG_TYPE,
                                                            getLine(), SkatType.STRING, type);
        }
        return getText();
    }

    /** Check if a field structure exists.
     * @param name field name
     * @return true if the field exists
     * @exception IllegalStateException if the node is not a {@link SkatType#STRUCTURE}
     */
    public boolean containsField(final String name)
        throws IllegalStateException {

        if (type != SkatType.STRUCTURE) {
            throw SkatException.createIllegalStateException(SkatMessages.WRONG_TYPE,
                                                            getLine(), SkatType.STRUCTURE, type);
        }

        for (int i = 0; i < getChildCount(); ++i) {
            final SkatParseTree child = (SkatParseTree) getChild(i);
            if (child.getName().equals(name)) {
                return true;
            }
        }

        return false;

    }

    /** Get a field structure.
     * @param name field name
     * @return field
     * @exception IllegalStateException if the node is not a {@link SkatType#STRUCTURE}
     * @exception NoSuchFieldException if the field cannot be found
     */
    public SkatParseTree getField(final String name)
        throws IllegalStateException, NoSuchFieldException {

        if (type != SkatType.STRUCTURE) {
            throw SkatException.createIllegalStateException(SkatMessages.WRONG_TYPE,
                                                            getLine(), SkatType.STRUCTURE, type);
        }

        for (int i = 0; i < getChildCount(); ++i) {
            final SkatParseTree child = (SkatParseTree) getChild(i);
            if (child.getName().equals(name)) {
                return child;
            }
        }

        throw SkatException.createNoSuchFieldException(SkatMessages.MISSING_INPUT_DATA, name);

    }

    /** Get the number of elements in an array.
     * @return number of elements in the array
     * @exception IllegalStateException if the node is not a {@link SkatType#ARRAY}
     */
    public int getElementsNumber()
        throws IllegalStateException {

        if (type != SkatType.ARRAY) {
            throw SkatException.createIllegalStateException(SkatMessages.WRONG_TYPE,
                                                            getLine(), SkatType.ARRAY, type);
        }

        return getChildCount();

    }

    /** Get an array element.
     * @param i array element index
     * @return array element
     * @exception IllegalStateException if the node is not a {@link SkatType#ARRAY}
     */
    public SkatParseTree getElement(final int i)
        throws IllegalStateException {

        if (type != SkatType.ARRAY) {
            throw SkatException.createIllegalStateException(SkatMessages.WRONG_TYPE,
                                                            getLine(), SkatType.ARRAY, type);
        }

        return (SkatParseTree) getChild(i);

    }

}
