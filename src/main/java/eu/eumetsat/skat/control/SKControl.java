/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;


/**
 * Interface representing a station-keeping control that should be achieved.
 * <p>
 * Station-keeping controls can be for example to ensure that spacecraft
 * remains within its allowed slot with as large margins as possible.
 * </p>
 * @see SKParametersList
 * @author Luc Maisonobe
 */
public interface SKControl extends Constrainable {

    /** Get the name of the control.
     * @return name of the control
     */
    String getName();

    /** Get the target value of the control.
     * @return target value of the control
     */
    double getTargetValue();

    /** Get the achieved value of the control.
     * <p>
     * The achieved value will change at the end
     * of each attempted simulation, depending
     * on the control parameters. The control
     * parameters will be set up by the station-keeping
     * engine in order to reach a value as close to the
     * {@link #getTarget target} as possible.
     * </p>
     * @return achieved value of the control
     */
    double getAchievedValue();

}
