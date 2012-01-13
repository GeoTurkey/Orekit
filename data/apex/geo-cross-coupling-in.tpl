# General simulation parameters
simulation       = {
    output_base_name        = "geo-cross-coupling";
    inertial_frame          = "GCRF";
    earth_frame             = "GMOD";
    solar_activity_strength = AVERAGE;
    start_date              = 2011-10-23T00:00:00.000;
    end_date                = 2012-10-21T00:00:00.000;
    output_step             = 43200.0;
    random_seed             = 156325253;
    ground_location         = {
        latitude     = 49.85;
        longitude    =  8.65;
        altitude     =  0.0
    };
    maneuvers               = [
        {
            name                  = "E/W";
            direction             = [ 1.0, 0.0, 0.0 ];
            thrust                = [ [  40, 0.0 ], [  30, 600.0 ] ];
            isp_curve             = [ [ 315, 0.0 ], [ 300, 600.0 ] ];
            dv_inf                = -0.3;
            dv_sup                =  0.3;
            dv_convergence        =  0.0001;
            dt_convergence        =  300.0;
            elimination_threshold =  0.01;
        },
        {
            name                  = "N/S";
            direction             = [ 0.0, 1.0, 0.0 ];
            thrust                = [ [  40, 0.0 ], [  30, 600.0 ] ];
            isp_curve             = [ [ 315, 0.0 ], [ 300, 600.0 ] ];
            dv_inf                = -15.0;
            dv_sup                =  15.0;
            dv_convergence        =  0.0001;
            dt_convergence        =  300.0;
            elimination_threshold =  0.1;
        }
    ]
};
# Array of initial states
initial_states   = [
    {
        name                   = "Meteosat7";
        bol_mass               = 1800;
        mass                   = 1800;
        cycle_number           = 1;
        orbit                  = {
            date               = 2011-10-23T01:47:06.165;
            orbit_type         = EQUINOCTIAL;
            angle_type         = MEAN;
            a                  = 42168449.623;
            ex                 = -6.66e-5;
            ey                 =  2.17e-5;
            hx                 =  0.0027459;
            hy                 = -0.0015674;
            longitude_argument =  0.153225665;
        }
    }
];
# Array of scenario components
scenario         = [
    {
        component           = ORBIT_DETERMINATION;
        managed_spacecrafts = [ "Meteosat7" ];
        orbit_type          = EQUINOCTIAL;
        angle_type          = MEAN;
        small               = 1.0e-12;
        correlation         = [
            [  1.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
            [  0.0, 1.0, 0.0, 0.0, 0.0, 0.0 ],
            [  0.0, 0.0, 1.0, 0.0, 0.0, 0.0 ],
            [  0.0, 0.0, 0.0, 1.0, 0.0, 0.0 ],
            [  0.0, 0.0, 0.0, 0.0, 1.0, 0.0 ],
            [  0.0, 0.0, 0.0, 0.0, 0.0, 1.0 ]
        ];
        standard_deviation  = [ 5.0, 1.0e-7, 1.0e-7, 1.0e-6, 1.0e-6, 1.0e-6 ];
    },
    {
        component             = CONTROL_LOOP;
        controlled_spacecraft = "Meteosat7";
        first_cycle           = 1;
        last_cycle            = 999;
        max_iterations        = 100;
        propagator            = {
            method                 = NUMERICAL;
            min_step               = 1.0;
            max_step               = 900.0;
            position_tolerance     = 100.0;
            gravity_field_degree   = 6;
            gravity_field_order    = 6;
            srp_cross_section      = 20.0;
            absorption_coefficient = 1.5;
            reflection_coefficient = 2.0;
            third_bodies           = [ SUN, MOON ];
        };
        controls              = [
            {
                type                        = INCLINATION_VECTOR;
                name                        = "inclination vector";
                maneuver_name               = "N/S";
                max_maneuvers               = 2;
                maneuvers_orbits_separation = 1;
                offset_first_maneuver       = 86400;
                reference_hx                =  0.0;
                reference_hy                = -1.5e-5;
                limit_inclination_angle     =  0.2;
                sampling                    = 3600;
                time_horizon                = 14.0;
            }
        ]
    },
    {
        component           = MANEUVER_CROSS_COUPLING;
        managed_spacecrafts = [ "Meteosat7" ];
        maneuver_name       = "N/S";
        nominal_direction   = [ 0.0, 1.0, 0.0 ];
        coupling_direction  = [ 1.0, 0.0, 0.0 ];
        coupling_ratio      = @cross_coupling_NS@;
    },
    {
        component                = CONTROL_LOOP;
        controlled_spacecraft    = "Meteosat7";
        first_cycle              = 1;
        last_cycle               = 999;
        max_iterations           = 100;
        propagator  = {
            method                 = NUMERICAL;
            min_step               = 1.0;
            max_step               = 900.0;
            position_tolerance     = 100.0;
            gravity_field_degree   = 6;
            gravity_field_order    = 6;
            srp_cross_section      = 20.0;
            absorption_coefficient = 1.5;
            reflection_coefficient = 2.0;
            third_bodies           = [ SUN, MOON ];
        };
        controls    = [
            {
                type                        = PARABOLIC_LONGITUDE;
                name                        = "parabolic longitude";
                maneuver_name               = "E/W";
                offset_first_maneuver       = 172800;
                max_maneuvers               = 3;
                maneuvers_orbits_separation = 1;
                east_longitude              = -56.9;
                west_longitude              = -57.1;
                sampling                    = 10800.0;
                time_horizon                = 14.0;
            }
        ]
    },
    {
        component           = MANEUVER_CROSS_COUPLING;
        managed_spacecrafts = [ "Meteosat7" ];
        maneuver_name       = "E/W";
        nominal_direction   = [ 1.0, 0.0, 0.0 ];
        coupling_direction  = [ 0.0, 0.0, 1.0 ];
        coupling_ratio      = @cross_coupling_EW@;
    },
    {
        component              = PROPAGATION;
        managed_spacecrafts    = [ "Meteosat7" ];
        long_burn_compensation = false;
        propagator             = {
            method                 = NUMERICAL;
            min_step               = 10.0;
            max_step               = 900.0;
            position_tolerance     = 50.0;
            gravity_field_degree   = 6;
            gravity_field_order    = 6;
            srp_cross_section      = 20.0;
            absorption_coefficient = 1.5;
            reflection_coefficient = 2.0;
            third_bodies           = [ SUN, MOON ];
        }
    }
]
