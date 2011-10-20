/* Copyright 2011 Eumetsat */
package eu.eumetsat.skat.utils;

/** Input parameter keys for SKAT.
 */
public enum ParameterKey {

    OUTPUT_FILE_NAME,

    INERTIAL_FRAME,
    EARTH_FRAME,
    GRAVITY_FIELD,
    GRAVITY_FIELD_DEGREE("degree"),
    GRAVITY_FIELD_ORDER("order"),

    ORBIT,

    ORBIT_CARTESIAN_DATE("date"),
    ORBIT_CARTESIAN_POSITION("position"),
    ORBIT_CARTESIAN_VELOCITY("velocity"),

    ORBIT_KEPLERIAN_DATE("date"),
    ORBIT_KEPLERIAN_A("a"),
    ORBIT_KEPLERIAN_E("e"),
    ORBIT_KEPLERIAN_I("i"),
    ORBIT_KEPLERIAN_PA("pa"),
    ORBIT_KEPLERIAN_RAAN("raan"),
    ORBIT_KEPLERIAN_MEAN_ANOMALY("M"),

    ORBIT_CIRCULAR_DATE("date"),
    ORBIT_CIRCULAR_A("a"),
    ORBIT_CIRCULAR_EX("ex"),
    ORBIT_CIRCULAR_EY("ey"),
    ORBIT_CIRCULAR_I("i"),
    ORBIT_CIRCULAR_RAAN("raan"),
    ORBIT_CIRCULAR_MEAN_LATITUDE_ARGUMENT("alpha_M"),

    ORBIT_EQUINOCTIAL_DATE("date"),
    ORBIT_EQUINOCTIAL_A("a"),
    ORBIT_EQUINOCTIAL_EX("ex"),
    ORBIT_EQUINOCTIAL_EY("ey"),
    ORBIT_EQUINOCTIAL_HX("hx"),
    ORBIT_EQUINOCTIAL_HY("hy"),
    ORBIT_EQUINOCTIAL_MEAN_LONGITUDE_ARGUMENT("lambda_M"),

    SIMULATION,
    SIMULATION_START_DATE("start_date"),
    SIMULATION_POSITION_TOLERANCE("position_tolerance"),
    SIMULATION_CYCLE_DURATION("duration"),
    SIMULATION_CYCLE_NUMBER("cycle_number");

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
