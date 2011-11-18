// seed for the random generator
seed                   = 987654321;

// we perform random drawings on one vector only, containing two uncorrelated variables
dispersed_vectors         = new Array("cross_coupling");
cross_coupling_keys       = new Array("cross_coupling_NS", "cross_coupling_EW");

// mean value for random drawings, here we use 0 to have centered repartition
cross_coupling_NS         = 0.0;
cross_coupling_EW         = 0.0;

// build zero matrix
cross_coupling_covariance_matrix       = new Array(cross_coupling_keys.length);
for (i = 0; i < cross_coupling_keys.length; ++i) {
    cross_coupling_covariance_matrix[i] = new Array(cross_coupling_keys.length);
    for (j = 0; j < cross_coupling_keys.length; ++j) {
        cross_coupling_covariance_matrix[i][j] = 0.0;
    }
}

// add variance (not standard deviation!) on diagonal elements
cross_coupling_covariance_matrix[0][0] = 0.05 * 0.05;
cross_coupling_covariance_matrix[1][1] = 0.02 * 0.02;

// on simulation output, we observe one vector containing the station keeping cost in plane and out of plane
observed_vectors = new Array("final_dv");
final_dv_keys    = new Array("final_in_plane_dv", "final_out_of_plane_dv");

