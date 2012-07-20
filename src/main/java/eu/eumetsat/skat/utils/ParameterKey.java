/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;



/** Input parameter keys for SKAT.
 */
public enum ParameterKey {

    SIMULATION("simulation"),
    SIMULATION_OUTPUT_BASE_NAME("output_base_name"),
    SIMULATION_INERTIAL_FRAME("inertial_frame"),
    SIMULATION_EARTH_FRAME("earth_frame"),
    SIMULATION_SOLAR_ACTIVITY_STRENGTH("solar_activity_strength"),
    SIMULATION_START_DATE("start_date"),
    SIMULATION_END_DATE("end_date"),
    SIMULATION_GROUND_LOCATION("ground_location"),
    SIMULATION_GROUND_LOCATION_LATITUDE("latitude"),
    SIMULATION_GROUND_LOCATION_LONGITUDE("longitude"),
    SIMULATION_GROUND_LOCATION_ALTITUDE("altitude"),
    SIMULATION_MANEUVERS("maneuvers"),
    SIMULATION_OUTPUT_STEP("output_step"),
    SIMULATION_RANDOM_SEED("random_seed"),

    ORBIT_TYPE,
    ANGLE_TYPE,

    INITIAL_STATES,
    INITIAL_STATE_NAME("name"),
    INITIAL_STATE_BOL_MASS("bol_mass"),
    INITIAL_STATE_CYCLE_NUMBER("cycle_number"),
    INITIAL_STATE_MANEUVERS("maneuvers"),
    INITIAL_STATE_MANEUVER_NAME("name"),
    INITIAL_STATE_MANEUVER_NUMBER("number"),
    INITIAL_STATE_MANEUVER_TOTAL_DV("total_dV"),
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
    COMPONENT_CONTROL_LOOP_MAX_ITER("max_iterations"),
    COMPONENT_CONTROL_LOOP_PROPAGATOR("propagator"),
    COMPONENT_CONTROL_LOOP_CONTROLS("controls"),

    CONTROL_TYPE("type"),
    CONTROL_NAME("name"),
    CONTROL_SAMPLING("sampling"),
    CONTROL_HORIZON("time_horizon"),
    CONTROL_MANEUVER_NAME("maneuver_name"),
    CONTROL_MAX_MANEUVERS("max_maneuvers"),
    CONTROL_MANEUVERS_ORBITS_SEPARATION("maneuvers_orbits_separation"),
    // Parabolic longitude
    CONTROL_PARABOLIC_FIRST_OFFSET("offset_first_maneuver"),
    CONTROL_PARABOLIC_LONGITUDE_EAST("east_longitude"),
    CONTROL_PARABOLIC_LONGITUDE_WEST("west_longitude"),
    // Eccentricity circle
    CONTROL_ECCENTRICITY_CIRCLE_CENTER_X("center_x"),
    CONTROL_ECCENTRICITY_CIRCLE_CENTER_Y("center_y"),
    CONTROL_ECCENTRICITY_CIRCLE_MEAN_RADIUS("mean_radius"),
    CONTROL_ECCENTRICITY_CIRCLE_MAX_RADIUS("max_radius"),
    CONTROL_ECCENTRICITY_CIRCLE_SINGLE_BURN("single_burn"),
    // Inclination vector
    CONTROL_INCLINATION_VECTOR_FIRST_OFFSET("offset_first_maneuver"),
    CONTROL_INCLINATION_VECTOR_REFERENCE_HX("reference_hx"),
    CONTROL_INCLINATION_VECTOR_REFERENCE_HY("reference_hy"),
    CONTROL_INCLINATION_LIMIT_INCLINATION_ANGLE("limit_inclination_angle"),
    // Local solar time
    CONTROL_SOLAR_TIME_FIRST_OFFSET("offset_first_maneuver"),
    CONTROL_SOLAR_TIME_LATITUDE("latitude"),
    CONTROL_SOLAR_TIME_ASCENDING("ascending"),
    CONTROL_SOLAR_TIME_SOLAR_TIME("solar_time"),
    CONTROL_SOLAR_TIME_SOLAR_TIME_TOLERANCE("solar_time_tolerance"),
    CONTROL_SOLAR_TIME_LONG_BURN_COMPENSATION("long_burn_compensation"),
    CONTROL_SOLAR_TIME_I_DRIFT_0("i_drift_0"),
    CONTROL_SOLAR_TIME_I_DRIFT_COS("i_drift_cos"),
    CONTROL_SOLAR_TIME_I_DRIFT_DOY("i_drift_doy"),
    CONTROL_SOLAR_TIME_PHASING_CYCLE_DAYS("phasing_cycle_days"),
    CONTROL_SOLAR_TIME_PHASING_CYCLE_ORBITS("phasing_cycle_orbits"),
    CONTROL_SOLAR_TIME_MANEUVERS_DOY("maneuvers_doy"),
    // Local solar time
    CONTROL_INCLINATION_FIRST_OFFSET("offset_first_maneuver"),
    CONTROL_INCLINATION_LATITUDE("latitude"),
    CONTROL_INCLINATION_ASCENDING("ascending"),
    CONTROL_INCLINATION_MEAN_VALUE("inclination_mean_value"),
    CONTROL_INCLINATION_TOLERANCE("inclination_tolerance"),
    CONTROL_INCLINATION_LONG_BURN_COMPENSATION("long_burn_compensation"),
    CONTROL_INCLINATION_I_DRIFT_0("i_drift_0"),
    CONTROL_INCLINATION_I_DRIFT_COS("i_drift_cos"),
    CONTROL_INCLINATION_I_DRIFT_DOY("i_drift_doy"),
    CONTROL_INCLINATION_PHASING_CYCLE_DAYS("phasing_cycle_days"),
    CONTROL_INCLINATION_PHASING_CYCLE_ORBITS("phasing_cycle_orbits"),
    CONTROL_INCLINATION_MANEUVERS_DOY("maneuvers_doy"),

    // In-plane ground track grid
    CONTROL_IN_PLANE_GROUND_TRACK_FIRST_OFFSET("offset_first_maneuver"),
    CONTROL_IN_PLANE_GROUND_TRACK_GRID_FILE("grid_file"),
    CONTROL_IN_PLANE_GROUND_TRACK_MAX_CROSS_TRACK_DISTANCE("max_cross_track_distance"),
    // Out-of-plane ground track grid
    CONTROL_OUT_OF_PLANE_GROUND_TRACK_FIRST_OFFSET("offset_first_maneuver"),
    CONTROL_OUT_OF_PLANE_GROUND_TRACK_GRID_FILE("grid_file"),
    CONTROL_OUT_OF_PLANE_GROUND_TRACK_MAX_CROSS_TRACK_DISTANCE("max_cross_track_distance"),
    CONTROL_OUT_OF_PLANE_GROUND_TRACK_GRID_BURN_COMPENSATION("long_burn_compensation"),

    // Maneuvers 
    MANEUVERS_NAME("name"),
    MANEUVERS_DIRECTION("direction"),
    MANEUVERS_THRUST("thrust"),
    MANEUVERS_ISP_CURVE("isp_curve"),
    MANEUVERS_DV_INF("dv_inf"),
    MANEUVERS_DV_SUP("dv_sup"),
    MANEUVERS_DV_CONVERGENCE("dv_convergence"),
    MANEUVERS_DT_CONVERGENCE("dt_convergence"),
    MANEUVERS_ELIMINATION_THRESHOLD("elimination_threshold"),

    COMPONENT_MANEUVER_DATE_ERROR_NAME("maneuver_name"),
    COMPONENT_MANEUVER_DATE_ERROR_STANDARD_DEVIATION("standard_deviation"),

    COMPONENT_MANEUVER_MAGNITUDE_ERROR_NAME("maneuver_name"),
    COMPONENT_MANEUVER_MAGNITUDE_ERROR_STANDARD_DEVIATION("standard_deviation"),

    COMPONENT_MISSED_MANEUVER_NAME("maneuver_name"),
    COMPONENT_MISSED_MANEUVER_THRESHOLD("miss_threshold"),
    COMPONENT_MISSED_MANEUVER_ORBITS_SEPARATION("orbits_separation"),

    COMPONENT_CROSS_COUPLING_NAME("maneuver_name"),
    COMPONENT_CROSS_COUPLING_NOMINAL_DIRECTION("nominal_direction"),
    COMPONENT_CROSS_COUPLING_COUPLING_DIRECTION("coupling_direction"),
    COMPONENT_CROSS_COUPLING_RATIO("coupling_ratio"),
    COMPONENT_CROSS_COUPLING_STANDARD_DEVIATION("standard_deviation"),

    COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_NAME("maneuver_name"),
    COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_ENTRY_DELAY("entry_delay"),
    COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_EXIT_DELAY("exit_delay"),
    COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_ORBITS_SEPARATION("orbits_separation"),
    COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_DURATION_MIN_RATIO("duration_min_ratio"),
    COMPONENT_MANEUVER_ECLIPSE_CONSTRAINT_NODE_ASYMMETRY_COMPENSATION("node_asymmetry_compensation"),

    COMPONENT_ONE_OR_TWO_BURNS_SPLIT_MANEUVER("split_maneuver"),

    COMPONENT_PROPAGATION_PROPAGATOR("propagator"),
    COMPONENT_PROPAGATION_METHOD("method"),
    COMPONENT_PROPAGATION_TRUNCATION_MANEUVER_NAME("truncation_maneuver_name"),
    COMPONENT_PROPAGATION_TRUNCATION_MANEUVER_DELAY("truncation_maneuver_delay"),

    // Numerical
    NUMERICAL_PROPAGATOR_MIN_STEP("min_step"),
    NUMERICAL_PROPAGATOR_MAX_STEP("max_step"),
    NUMERICAL_PROPAGATOR_TOLERANCE("position_tolerance"),
    NUMERICAL_PROPAGATOR_GRAVITY_FIELD_DEGREE("gravity_field_degree"),
    NUMERICAL_PROPAGATOR_GRAVITY_FIELD_ORDER("gravity_field_order"),
    NUMERICAL_PROPAGATOR_DRAG_STANDARD_DEVIATION("drag_standard_deviation"),
    NUMERICAL_PROPAGATOR_DRAG_CROSS_SECTION("drag_cross_section"),
    NUMERICAL_PROPAGATOR_DRAG_COEFFICIENT("drag_coefficient"),
    NUMERICAL_PROPAGATOR_SRP_STANDARD_DEVIATION("srp_standard_deviation"),
    NUMERICAL_PROPAGATOR_SRP_CROSS_SECTION("srp_cross_section"),
    NUMERICAL_PROPAGATOR_ABSORPTION_COEFFICIENT("absorption_coefficient"),
    NUMERICAL_PROPAGATOR_REFLECTION_COEFFICIENT("reflection_coefficient"),
    NUMERICAL_PROPAGATOR_THIRD_BODIES("third_bodies"),
    // DSST
    DSST_PROPAGATOR_MIN_STEP("min_step"),
    DSST_PROPAGATOR_MAX_STEP("max_step"),
    DSST_PROPAGATOR_TOLERANCE("position_tolerance"),
    DSST_PROPAGATOR_GRAVITY_FIELD_DEGREE("gravity_field_degree"),
    DSST_PROPAGATOR_GRAVITY_FIELD_ORDER("gravity_field_order"),
    DSST_PROPAGATOR_DRAG_STANDARD_DEVIATION("drag_standard_deviation"),
    DSST_PROPAGATOR_DRAG_CROSS_SECTION("drag_cross_section"),
    DSST_PROPAGATOR_DRAG_COEFFICIENT("drag_coefficient"),
    DSST_PROPAGATOR_SRP_STANDARD_DEVIATION("srp_standard_deviation"),
    DSST_PROPAGATOR_SRP_CROSS_SECTION("srp_cross_section"),
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
