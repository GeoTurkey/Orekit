/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.control;

import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.sampling.OrekitStepHandler;


/**
 * Interface representing a station-keeping control that should be achieved.
 * <p>
 * Station-keeping controls can be for example to ensure that spacecraft
 * remains within its allowed slot with as large margins as possible.
 * </p>
 * @see SKParameter
 * @author Luc Maisonobe
 */
public interface SKControl extends SKElement {

    /** Get the name of the control.
     * @return name of the control
     */
    String getName();

    /** Get the scale of the control.
     * <p>
     * The scale of the control is a scalar parameter with the same physical unit
     * as the control itself (radians, meters, seconds ...). It's purpose is to
     * allow mixing controls in a global scalar objective function by computing<br>
     *   J = &sum;(((control<sub>i</sub>.{@link SKControl#getAchievedValue() getAchievedValue()}
     *             - control<sub>i</sub>.{@link SKControl#getTargetValue() getTargetValue()})
     *             / scale<sub>i</sub>)<sup>2</sup>)<br>
     * the sum being computed across all scaled controls.
     * </p>
     * @return scale of the control
     */
    double getScale();

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

    /** Get the name of the controlled spacecraft.
     * @return name of the controlled spacecraft.
     */
    String getControlledSpacecraftName();

    /** Get the name of the reference spacecraft.
     * @return name of the reference spacecraft
     * (null if control law does not depend on a reference spacecraft).
     */
    String getReferenceSpacecraftName();

    /** Get the event detector associated with this element.
     * @return event detector associated with this element (may be null)
     */
    EventDetector getEventDetector();

    /** Get the step handler associated with this element.
     * @return step handler associated with this element (may be null)
     */
    OrekitStepHandler getStepHandler();

}
