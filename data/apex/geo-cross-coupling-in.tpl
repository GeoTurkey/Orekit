# General simulation parameters
simulation       = {
    output_base_name = "geo-cross-coupling";
    inertial_frame   = "EME2000";
    earth_frame      = "ITRF2008 with tides";
    start_date       = 2011-10-23T00:00:00.000;
    end_date         = 2011-12-01T00:00:00.000;
    cycle_duration   = 14.0;
    rolling_cycles   = 2;
    ground_location  = {
        latitude     = 49.85;
        longitude    =  8.65;
        altitude     =  0.0
    };
    output_step      = 21600.0;
    random_seed      = 156325253;
};

# Array of initial states
# there must be exactly one element for each spacecraft
initial_states   = [
    {
        name                   = "Meteosat 7";
        bol_mass               = 1800;
        cycle_number           = 1;
        in_plane_maneuvers     = 0;
        in_plane_total_dV      = 0.0;
        out_of_plane_maneuvers = 0;
        out_of_plane_total_dV  = 0.0;
        mass                   = 1200.0;

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
# each scenario component specifies to which spacecrafts it applies
scenario         = [
    {
        component           = ORBIT_DETERMINATION;
        managed_spacecrafts = [ "Meteosat 7" ];
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
        component                = CONTROL_LOOP;
        controlled_spacecraft    = "Meteosat 7";
        first_cycle              = 1;
        last_cycle               = 999;
        max_eval                 = 1000;
        global_stop_criterion    = 0.001;
        in_plane_elimination     = 0.001;
        out_of_plane_elimination = 0.01;
        optimizer             = {
            method                     = NELDER_MEAD;
            initial_simplex_size_ratio = 0.01
        };
        propagator  = {
            method                 = NUMERICAL;
            min_step               = 1.0;
            max_step               = 900.0;
            position_tolerance     = 100.0;
            gravity_field_degree   = 6;
            gravity_field_order    = 6;
            cross_section          = 20.0;
            absorption_coefficient = 1.5;
            reflection_coefficient = 2.0;
            third_bodies           = [ SUN, MOON ];
        };
        controls    = [
            {
                scaling_divisor     = 1.0e-4;
                type                = INCLINATION_VECTOR;
                name                = "inclination vector";
                target_x            = 9.0e-4;
                target_y            =  0.0;
                limit_circle_radius = 8.45e-4;
                sampling            = 7200.0;
            }
        ];
        maneuvers   = [
            {
                in_plane             = false;
                relative_to_previous = false;
                name                 = "N/S";
                direction            = [ 0.0, 1.0, 0.0 ];
                thrust               = 40;
                isp_curve            = [ [ 315, 0.0 ], [ 300, 600.0 ] ];
                dv_min               = -3.0;
                dv_max               =  3.0;
                dv_convergence       =  0.01;
                nominal_date         = 172800;
                dt_min               = -43200;
                dt_max               =  43200;
                dt_convergence       =  60.0;
            }
        ];
    },
    {
        component                = CONTROL_LOOP;
        controlled_spacecraft    = "Meteosat 7";
        first_cycle              = 1;
        last_cycle               = 999;
        max_eval                 = 1000;
        global_stop_criterion    = 0.001;
        in_plane_elimination     = 0.001;
        out_of_plane_elimination = 0.01;
        optimizer             = {
            method                     = NELDER_MEAD;
            initial_simplex_size_ratio = 0.01
        };
        propagator  = {
            method                 = NUMERICAL;
            min_step               = 1.0;
            max_step               = 900.0;
            position_tolerance     = 100.0;
            gravity_field_degree   = 6;
            gravity_field_order    = 6;
            cross_section          = 20.0;
            absorption_coefficient = 1.5;
            reflection_coefficient = 2.0;
            third_bodies           = [ SUN, MOON ];
        };
        controls    = [
            {
                scaling_divisor  = 0.1;
                type             = CENTERED_LONGITUDE;
                name             = "centered longitude";
                east_longitude   =  -57.1;
                west_longitude   =  -56.9;
                sampling         = 7200.0;
            },
            {
                scaling_divisor  = 1.0e-4;
                type             = ECCENTRICITY_CIRCLE;
                name             = "eccentricity circle";
                center_x         = -1.5e-4;
                center_y         =  0.0;
                radius           = 2.0e-4;
                sampling         = 7200.0;
            }
        ];
        maneuvers   = [
            {
                in_plane             = true;
                relative_to_previous = false;
                name                 = "E/W-1";
                direction            = [ 1.0, 0.0, 0.0 ];
                thrust               = 40;
                isp_curve            = [ [ 315, 0.0 ], [ 300, 600.0 ] ];
                dv_min               = -0.3;
                dv_max               =  0.3;
                dv_convergence       =  0.01;
                nominal_date         = 345600;
                dt_min               = -43200;
                dt_max               =  43200;
                dt_convergence       =  60.0;
            },
            {
                in_plane             = true;
                relative_to_previous = true;
                name                 = "E/W-2";
                direction            = [ 1.0, 0.0, 0.0 ];
                thrust               = 40;
                isp_curve            = [ [ 315, 0.0 ], [ 300, 600.0 ] ];
                dv_min               = -0.3;
                dv_max               =  0.3;
                dv_convergence       =  0.01;
                nominal_date         = 43200;
                dt_min               =    0.0;
                dt_max               =    0.0;
                dt_convergence       =  60.0;
            }
        ];
    },
    {
        component           = MANEUVER_DATE_ERROR;
        managed_spacecrafts = [ "Meteosat 7" ];
        in_plane            = true;
        out_of_plane        = true;
        standard_deviation  = 10.0;
    },
    {
        component           = MANEUVER_MAGNITUDE_ERROR;
        managed_spacecrafts = [ "Meteosat 7" ];
        in_plane            = false;
        out_of_plane        = true;
        standard_deviation  = 0.01;
    },
    {
        component           = MANEUVER_MAGNITUDE_ERROR;
        managed_spacecrafts = [ "Meteosat 7" ];
        in_plane            = true;
        out_of_plane        = false;
        standard_deviation  = 0.02;
    },
    {
        component           = MISSED_MANEUVER;
        managed_spacecrafts = [ "Meteosat 7" ];
        in_plane            = true;
        out_of_plane        = true;
        miss_threshold      = 0.1;
        orbits_separation   = 1;
    },
    {
        component           = MANEUVER_CROSS_COUPLING;
        managed_spacecrafts = [ "Meteosat 7" ];
        in_plane            = true;
        out_of_plane        = false;
        nominal_direction   = [ 1.0, 0.0, 0.0 ];
        coupling_direction  = [ 0.0, 0.0, 1.0 ];
        coupling_ratio      = @cross_coupling_EW@;
    },
    {
        component           = MANEUVER_CROSS_COUPLING;
        managed_spacecrafts = [ "Meteosat 7" ];
        in_plane            = false;
        out_of_plane        = true;
        nominal_direction   = [ 0.0, 1.0, 0.0 ];
        coupling_direction  = [ 1.0, 0.0, 0.0 ];
        coupling_ratio      = @cross_coupling_NS@;
    },
    {
        component              = PROPAGATION;
        managed_spacecrafts    = [ "Meteosat 7" ];
        long_burn_compensation = false;
        propagator             = {
            method                 = NUMERICAL;
            min_step               = 10.0;
            max_step               = 900.0;
            position_tolerance     = 50.0;
            gravity_field_degree   = 10;
            gravity_field_order    = 10;
            cross_section          = 20.0;
            absorption_coefficient = 1.5;
            reflection_coefficient = 2.0;
            third_bodies           = [ SUN, MOON ];
        }
    }
];

monitoring_mono = [
    IN_PLANE_MANEUVER_TOTAL_DV,
    OUT_OF_PLANE_MANEUVER_TOTAL_DV,
]
