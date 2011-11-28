/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;


/** Input parameter keys for SKAT.
 */
public enum ParameterKey {

    SIMULATION,
    SIMULATION_OUTPUT_BASE_NAME("output_base_name"),
    SIMULATION_INERTIAL_FRAME("inertial_frame"),
    SIMULATION_EARTH_FRAME("earth_frame"),
    SIMULATION_START_DATE("start_date"),
    SIMULATION_END_DATE("end_date"),
    SIMULATION_CYCLE_DURATION("cycle_duration"),
    SIMULATION_ROLLING_CYCLES("rolling_cycles"),
    SIMULATION_GROUND_LOCATION("ground_location"),
    SIMULATION_GROUND_LOCATION_LATITUDE("latitude"),
    SIMULATION_GROUND_LOCATION_LONGITUDE("longitude"),
    SIMULATION_GROUND_LOCATION_ALTITUDE("altitude"),
    SIMULATION_OUTPUT_STEP("output_step"),
    SIMULATION_RANDOM_SEED("random_seed"),

    ORBIT_TYPE,
    ANGLE_TYPE,

    INITIAL_STATES,
    INITIAL_STATE_NAME("name"),
    INITIAL_STATE_BOL_MASS("bol_mass"),
    INITIAL_STATE_CYCLE_NUMBER("cycle_number"),
    INITIAL_STATE_IN_PLANE_MANEUVERS("in_plane_maneuvers"),
    INITIAL_STATE_IN_PLANE_TOTAL_DV("in_plane_total_dV"),
    INITIAL_STATE_OUT_OF_PLANE_MANEUVERS("out_of_plane_maneuvers"),
    INITIAL_STATE_OUT_OF_PLANE_TOTAL_DV("out_of_plane_total_dV"),
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

    ORBIT_DETERMINATION_CORRELATION("correlation"),
    ORBIT_DETERMINATION_STANDARD_DEVIATION("standard_deviation"),
    ORBIT_DETERMINATION_SMALL("small"),

    COMPONENT_CONTROL_LOOP_CONTROLLED_SPACECRAFT("controlled_spacecraft"),
    COMPONENT_CONTROL_LOOP_FIRST_CYCLE("first_cycle"),
    COMPONENT_CONTROL_LOOP_LAST_CYCLE("last_cycle"),
    COMPONENT_CONTROL_LOOP_MAX_EVAL("max_eval"),
    COMPONENT_CONTROL_LOOP_IN_PLANE_ELIMINATION("in_plane_elimination"),
    COMPONENT_CONTROL_LOOP_OUT_OF_PLANE_ELIMINATION("out_of_plane_elimination"),
    COMPONENT_CONTROL_LOOP_GLOBAL_STOP_CRITERION("global_stop_criterion"),
    COMPONENT_CONTROL_LOOP_OPTIMIZER("optimizer"),
    OPTIMIZER_METHOD("method"),
    NELDER_MEAD_INITIAL_SIMPLEX_SIZE_RATIO("initial_simplex_size_ratio"),
    CMAES_POPULATION_SIZE("population_size"),
    CMAES_MAX_ITERATIONS("max_iterations"),
    COMPONENT_CONTROL_LOOP_PROPAGATOR("propagator"),
    COMPONENT_CONTROL_LOOP_CONTROLS("controls"),
    COMPONENT_CONTROL_LOOP_MANEUVERS("maneuvers"),

    CONTROL_SCALING_DIVISOR("scaling_divisor"),
    CONTROL_TYPE("type"),
    CONTROL_NAME("name"),
    CONTROL_SAMPLING("sampling"),
    CONTROL_MINIMIZED_MANEUVERS_IN_PLANE("in_plane"),
    CONTROL_MINIMIZED_MANEUVERS_OUT_OF_PLANE("out_of_plane"),
    CONTROL_CENTERED_LONGITUDE_EAST("east_longitude"),
    CONTROL_CENTERED_LONGITUDE_WEST("west_longitude"),
    CONTROL_ECCENTRICITY_CIRCLE_CENTER_X("center_x"),
    CONTROL_ECCENTRICITY_CIRCLE_CENTER_Y("center_y"),
    CONTROL_ECCENTRICITY_CIRCLE_RADIUS("radius"),
    CONTROL_INCLINATION_VECTOR_TARGET_X("target_x"),
    CONTROL_INCLINATION_VECTOR_TARGET_Y("target_y"),
    CONTROL_INCLINATION_LIMIT_CIRCLE_RADIUS("limit_circle_radius"),
    CONTROL_SOLAR_TIME_LATITUDE("latitude"),
    CONTROL_SOLAR_TIME_ASCENDING("ascending"),
    CONTROL_SOLAR_TIME_SOLAR_TIME("solar_time"),
    CONTROL_GROUND_TRACK_LATITUDE("latitude"),
    CONTROL_GROUND_TRACK_LONGITUDE("longitude"),
    CONTROL_GROUND_TRACK_ASCENDING("ascending"),
    CONTROL_GROUND_TRACK_ORBITS_PER_CYCLE("orbits_per_cycle"),
    CONTROL_GROUND_TRACK_DAYS_PER_CYCLE("days_per_cycle"),

    MANEUVERS_IN_PLANE("in_plane"),
    MANEUVERS_RELATIVE_TO_PREVIOUS("relative_to_previous"),
    MANEUVERS_NAME("name"),
    MANEUVERS_DIRECTION("direction"),
    MANEUVERS_THRUST("thrust"),
    MANEUVERS_ISP_CURVE("isp_curve"),
    MANEUVERS_DV_MIN("dv_min"),
    MANEUVERS_DV_MAX("dv_max"),
    MANEUVERS_DV_CONVERGENCE("dv_convergence"),
    MANEUVERS_NOMINAL_DATE("nominal_date"),
    MANEUVERS_DT_MIN("dt_min"),
    MANEUVERS_DT_MAX("dt_max"),
    MANEUVERS_DT_CONVERGENCE("dt_convergence"),

    COMPONENT_MANEUVER_DATE_ERROR_IN_PLANE("in_plane"),
    COMPONENT_MANEUVER_DATE_ERROR_OUT_OF_PLANE("out_of_plane"),
    COMPONENT_MANEUVER_DATE_ERROR_STANDARD_DEVIATION("standard_deviation"),

    COMPONENT_MANEUVER_MAGNITUDE_ERROR_IN_PLANE("in_plane"),
    COMPONENT_MANEUVER_MAGNITUDE_ERROR_OUT_OF_PLANE("out_of_plane"),
    COMPONENT_MANEUVER_MAGNITUDE_ERROR_STANDARD_DEVIATION("standard_deviation"),

    COMPONENT_MISSED_MANEUVER_IN_PLANE("in_plane"),
    COMPONENT_MISSED_MANEUVER_OUT_OF_PLANE("out_of_plane"),
    COMPONENT_MISSED_MANEUVER_THRESHOLD("miss_threshold"),
    COMPONENT_MISSED_MANEUVER_RESCHEDULING_DELAY("rescheduling_delay"),

    COMPONENT_CROSS_COUPLING_IN_PLANE("in_plane"),
    COMPONENT_CROSS_COUPLING_OUT_OF_PLANE("out_of_plane"),
    COMPONENT_CROSS_COUPLING_NOMINAL_DIRECTION("nominal_direction"),
    COMPONENT_CROSS_COUPLING_COUPLING_DIRECTION("coupling_direction"),
    COMPONENT_CROSS_COUPLING_RATIO("coupling_ratio"),

    COMPONENT_MANEUVER_SPLITTER_IN_PLANE("in_plane"),
    COMPONENT_MANEUVER_SPLITTER_OUT_OF_PLANE("out_of_plane"),
    COMPONENT_MANEUVER_SPLITTER_MAX_DV("max_dv"),
    COMPONENT_MANEUVER_SPLITTER_MIN_DT("min_dt"),

    COMPONENT_PROPAGATION_PROPAGATOR("propagator"),
    COMPONENT_PROPAGATION_METHOD("method"),
    COMPONENT_PROPAGATION_LONG_BURN_COMPENSATION("long_burn_compensation"),
    NUMERICAL_PROPAGATOR_MIN_STEP("min_step"),
    NUMERICAL_PROPAGATOR_MAX_STEP("max_step"),
    NUMERICAL_PROPAGATOR_TOLERANCE("position_tolerance"),
    NUMERICAL_PROPAGATOR_GRAVITY_FIELD_DEGREE("gravity_field_degree"),
    NUMERICAL_PROPAGATOR_GRAVITY_FIELD_ORDER("gravity_field_order"),
    NUMERICAL_PROPAGATOR_CROSS_SECTION("cross_section"),
    NUMERICAL_PROPAGATOR_DRAG_COEFFICIENT("drag_coefficient"),
    NUMERICAL_PROPAGATOR_ABSORPTION_COEFFICIENT("absorption_coefficient"),
    NUMERICAL_PROPAGATOR_REFLECTION_COEFFICIENT("reflection_coefficient"),
    NUMERICAL_PROPAGATOR_THIRD_BODIES("third_bodies"),
    DSST_PROPAGATOR_MIN_STEP("min_step"),
    DSST_PROPAGATOR_MAX_STEP("max_step"),
    DSST_PROPAGATOR_TOLERANCE("position_tolerance"),
    DSST_PROPAGATOR_GRAVITY_FIELD_DEGREE("gravity_field_degree"),
    DSST_PROPAGATOR_GRAVITY_FIELD_ORDER("gravity_field_order"),
    DSST_PROPAGATOR_CROSS_SECTION("cross_section"),
    DSST_PROPAGATOR_DRAG_COEFFICIENT("drag_coefficient"),
    DSST_PROPAGATOR_ABSORPTION_COEFFICIENT("absorption_coefficient"),
    DSST_PROPAGATOR_REFLECTION_COEFFICIENT("reflection_coefficient"),
    DSST_PROPAGATOR_THIRD_BODIES("third_bodies"),

    MONITORING_MONO,
    MONITORING_DUO;

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
