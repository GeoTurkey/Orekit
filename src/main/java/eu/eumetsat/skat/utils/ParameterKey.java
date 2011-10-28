/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

/** Input parameter keys for SKAT.
 */
public enum ParameterKey {

    OUTPUT_FILE_NAME,

    SIMULATION,
    SIMULATION_INERTIAL_FRAME("inertial_frame"),
    SIMULATION_EARTH_FRAME("earth_frame"),
    SIMULATION_START_DATE("start_date"),
    SIMULATION_END_DATE("end_date"),
    SIMULATION_CYCLE_DURATION("cycle_duration"),
    SIMULATION_ROLLING_CYCLES("rolling_cycles"),
    SIMULATION_OUTPUT_STEP("output_step"),
    SIMULATION_RANDOM_SEED("random_seed"),

    ORBIT_TYPE,
    ANGLE_TYPE,

    INITIAL_STATES,
    INITIAL_STATE_NAME("name"),
    INITIAL_STATE_BOL_MASS("bol_mass"),
    INITIAL_STATE_CYCLE_NUMBER("cycle_number"),
    INITIAL_STATE_IN_PLANE_MANEUVERS("in_plane_maneuvers"),
    INITIAL_STATE_IN_PLANE_DV("in_plane_dV"),
    INITIAL_STATE_OUT_OF_PLANE_MANEUVERS("out_of_plane_maneuvers"),
    INITIAL_STATE_OUT_OF_PLANE_DV("out_of_plane_dV"),
    INITIAL_STATE_MASS("mass"),
    INITIAL_STATE_ORBIT("orbit"),

    ORBIT_CARTESIAN_DATE("date"),
    ORBIT_CARTESIAN_POSITION("position"),
    ORBIT_CARTESIAN_VELOCITY("velocity"),

    ORBIT_KEPLERIAN_DATE("date"),
    ORBIT_KEPLERIAN_A("a"),
    ORBIT_KEPLERIAN_E("e"),
    ORBIT_KEPLERIAN_I("i"),
    ORBIT_KEPLERIAN_PA("pa"),
    ORBIT_KEPLERIAN_RAAN("raan"),
    ORBIT_KEPLERIAN_ANOMALY("anomaly"),

    ORBIT_CIRCULAR_DATE("date"),
    ORBIT_CIRCULAR_A("a"),
    ORBIT_CIRCULAR_EX("ex"),
    ORBIT_CIRCULAR_EY("ey"),
    ORBIT_CIRCULAR_I("i"),
    ORBIT_CIRCULAR_RAAN("raan"),
    ORBIT_CIRCULAR_LATITUDE_ARGUMENT("latitude_argument"),

    ORBIT_EQUINOCTIAL_DATE("date"),
    ORBIT_EQUINOCTIAL_A("a"),
    ORBIT_EQUINOCTIAL_EX("ex"),
    ORBIT_EQUINOCTIAL_EY("ey"),
    ORBIT_EQUINOCTIAL_HX("hx"),
    ORBIT_EQUINOCTIAL_HY("hy"),
    ORBIT_EQUINOCTIAL_LONGITUDE_ARGUMENT("longitude_argument"),

    SCENARIO,
    COMPONENT_TYPE("component"),
    COMPONENT_MANAGED_SPACECRAFTS("managed_spacecrafts"),

    ORBIT_DETERMINATION_COVARIANCE("covariance"),
    ORBIT_DETERMINATION_SMALL("small"),

    COMPONENT_CONTROL_LOOP_OPTIMIZER("optimizer"),
    COMPONENT_CONTROL_LOOP_FIRST_CYCLE("first_cycle"),
    COMPONENT_CONTROL_LOOP_LAST_CYCLE("last_cycle"),
    COMPONENT_CONTROL_LOOP_MAX_EVAL("max_eval"),
    COMPONENT_CONTROL_LOOP_NB_POINTS("nb_points"),
    COMPONENT_CONTROL_LOOP_PROPAGATOR("propagator"),
    COMPONENT_CONTROL_LOOP_CONTROLS("controls"),
    COMPONENT_CONTROL_LOOP_MANEUVERS("maneuvers"),

    CONTROL_SCALE("scale"),
    CONTROL_TYPE("type"),
    CONTROL_NAME("name"),
    CONTROL_SAMPLING("sampling"),
    CONTROL_LONGITUDE_MARGINS_EAST("east_boundary"),
    CONTROL_LONGITUDE_MARGINS_WEST("west_boundary"),
    CONTROL_LONGITUDE_MARGINS_TARGET("target"),
    CONTROL_ECCENTRICITY_CIRCLE_CENTER_X("center_x"),
    CONTROL_ECCENTRICITY_CIRCLE_CENTER_Y("center_y"),
    CONTROL_ECCENTRICITY_CIRCLE_RADIUS("radius"),

    MANEUVERS_IN_PLANE("in_plane"),
    MANEUVERS_NAME("name"),
    MANEUVERS_DIRECTION("direction"),
    MANEUVERS_ISP_CURVE("isp_curve"),
    MANEUVERS_DV_MIN("dv_min"),
    MANEUVERS_DV_MAX("dv_max"),
    MANEUVERS_NOMINAL_DATE("nominal_date"),
    MANEUVERS_DT_MIN("dt_min"),
    MANEUVERS_DT_MAX("dt_max"),

    COMPONENT_MANEUVER_DATE_ERROR_IN_PLANE("in_plane"),
    COMPONENT_MANEUVER_DATE_ERROR_OUT_OF_PLANE("out_of_plane"),
    COMPONENT_MANEUVER_DATE_ERROR_STANDARD_DEVIATION("standard_deviation"),

    COMPONENT_MANEUVER_MAGNITUDE_ERROR_IN_PLANE("in_plane"),
    COMPONENT_MANEUVER_MAGNITUDE_ERROR_OUT_OF_PLANE("out_of_plane"),
    COMPONENT_MANEUVER_MAGNITUDE_ERROR_STANDARD_DEVIATION("standard_deviation"),

    COMPONENT_PROPAGATION_PROPAGATOR("propagator"),
    COMPONENT_PROPAGATION_METHOD("method"),
    NUMERICAL_PROPAGATOR_TOLERANCE("position_tolerance"),
    NUMERICAL_PROPAGATOR_GRAVITY_FIELD_DEGREE("gravity_field_degree"),
    NUMERICAL_PROPAGATOR_GRAVITY_FIELD_ORDER("gravity_field_order");

    /** Key to be recognized in input files. */
    private final String key;

    /** Simple constructor, using the lowercase name as the key.
     */
    private ParameterKey() {
        this.key = toString().toLowerCase();
    }

    /** Simple constructor.
     * @param key key to be recognized in input files
     */
    private ParameterKey(final String key) {
        this.key = key;
    }

    /** Get the key.
     * @return key to be recognized in input files
     */
    public String getKey() {
        return key;
    }

}
