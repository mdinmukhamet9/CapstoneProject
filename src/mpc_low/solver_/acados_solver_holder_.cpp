#include "acados_solver_holder.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

my_NMPC_solver::my_NMPC_solver(int n) {
    num_steps = n;
    Jackal_solver_capsule *my_acados_ocp_capsule = Jackal_acados_create_capsule();
    acados_ocp_capsule = my_acados_ocp_capsule;
    int N = JACKAL_N;
    double* new_time_steps = NULL;
    int status = Jackal_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);
    if (status) {
        printf("Jackal_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }
}

int my_NMPC_solver::solve_my_mpc(double current_robot_position[3], double costmap_data[520], 
                                double current_robot_goal[3], double tracking_goal[30], 
                                double results[8], double trajectory[33]) {
    printf("\n--- MPC Inputs ---\n");
    printf("Start: [%.3f, %.3f, %.3f]\n", current_robot_position[0], current_robot_position[1], current_robot_position[2]);
    printf("Goal (relative): [%.3f, %.3f, %.3f]\n", current_robot_goal[0], current_robot_goal[1], current_robot_goal[2]);

    double absolute_goal[3] = {
        current_robot_position[0] + current_robot_goal[0],
        current_robot_position[1] + current_robot_goal[1],
        current_robot_goal[2]
    };

    int status = -1;
    int N = JACKAL_N;

    ocp_nlp_config *nlp_config = Jackal_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = Jackal_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = Jackal_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = Jackal_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = Jackal_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = Jackal_acados_get_nlp_opts(acados_ocp_capsule);

    int idxbx0[NBX0], idxbx[NBX], idxbu[NBU], idxsh[NSH];
    for (int i=0; i<NBX0; i++) idxbx0[i] = i;
    for (int i=0; i<NBX; i++) idxbx[i] = i;
    for (int i=0; i<NBU; i++) idxbu[i] = i;
    for (int i=0; i<NSH; i++) idxsh[i] = i;

    double lbx0[NBX0], ubx0[NBX0];
    for(int i=0; i<NBX0; i++) {
        lbx0[i] = current_robot_position[i]-1e-3;
        ubx0[i] = current_robot_position[i]+1e-3;
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    double pi = 3.1415926;
    double lbx[3] = {-10.0, -10.0, -6*pi};
    double ubx[3] = {+10.0, +10.0, 6*pi};
    double lbu[2] = {-0.5, -1.0};
    double ubu[2] = {0.5, 1.0};

    double y_ref[NY] = {0.0};
    y_ref[0] = absolute_goal[0];
    y_ref[1] = absolute_goal[1];
    y_ref[2] = absolute_goal[2];
    y_ref[3] = 0.25; // Encourage positive linear velocity
    y_ref[4] = 0.0;

    double y_ref_N[NYN] = {0.0};
    for(int i=0; i<NYN; i++) y_ref_N[i] = y_ref[i];

    printf("Reference trajectory (absolute): [%.3f, %.3f, %.3f]\n", y_ref[0], y_ref[1], y_ref[2]);

    double W[(NU+NX)*(NU+NX)] = {0.0};
    double WN[NX*NX] = {0.0};

    // Balanced weights to reduce oscillation
    W[0] = 5.0;  // X
    W[1*(NU+NX)+1] = 10.0; // Y
    W[2*(NU+NX)+2] = 25.0;  // Yaw
    W[3*(NU+NX)+3] = 15.0; // Linear velocity (increased to penalize negative velocities)
    W[4*(NU+NX)+4] = 15.0;  // Angular velocity

    WN[0] = 20.0;
    WN[NX+1] = 20.0;
    WN[2*NX+2] = 50.0; // Terminal yaw weight

    // Add obstacle avoidance using costmap
    double costmap_resolution = 0.05;
    int costmap_width = 26;
    int costmap_height = 20;
    double costmap_origin_x = -costmap_width * costmap_resolution / 2.0;
    double costmap_origin_y = -costmap_height * costmap_resolution / 2.0;

    double lsh[NSH] = {0.0}, ush[NSH] = {0.0};
    double obstacle_weight = 10.0;

    for(int stage = 0; stage < N; stage++) {
        double x_pred = current_robot_position[0] + tracking_goal[stage * 3];
        double y_pred = current_robot_position[1] + tracking_goal[stage * 3 + 1];
        if (tracking_goal[stage * 3] == 0.0) {
            double alpha = (double)stage / N;
            x_pred = current_robot_position[0] + alpha * current_robot_goal[0];
            y_pred = current_robot_position[1] + alpha * current_robot_goal[1];
        }

        int idx_x = static_cast<int>((x_pred - costmap_origin_x) / costmap_resolution);
        int idx_y = static_cast<int>((y_pred - costmap_origin_y) / costmap_resolution);

        if (idx_x >= 0 && idx_x < costmap_width && idx_y >= 0 && idx_y < costmap_height) {
            int costmap_idx = idx_y * costmap_width + idx_x;
            double cost = costmap_data[costmap_idx];
            if (cost > 0) {
                double scaled_cost = (cost / 255.0) * obstacle_weight;
                lsh[0] = -scaled_cost;
                ush[0] = scaled_cost;
                ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "lsh", lsh);
                ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "ush", ush);
                ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "idxsh", idxsh);
            }
        }

        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "W", W);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "yref", y_ref);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", WN);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", y_ref_N);

    // Improved initial guess
    double x_init[NX*(N+1)];
    for(int i=0; i<=N; i++) {
        double alpha = (double)i/N;
        double interpolated_x = current_robot_position[0] + alpha * current_robot_goal[0];
        double interpolated_y = current_robot_position[1] + alpha * current_robot_goal[1];
        double interpolated_yaw = current_robot_position[2] + alpha * (current_robot_goal[2] - current_robot_position[2]);

        if (i * 3 < 30 && tracking_goal[i * 3] != 0.0) {
            double tracking_x = current_robot_position[0] + tracking_goal[i * 3];
            double tracking_y = current_robot_position[1] + tracking_goal[i * 3 + 1];
            double tracking_yaw = tracking_goal[i * 3 + 2];

            double blend_factor = 0.7;
            x_init[i*NX]   = blend_factor * tracking_x + (1.0 - blend_factor) * interpolated_x;
            x_init[i*NX+1] = blend_factor * tracking_y + (1.0 - blend_factor) * interpolated_y;
            x_init[i*NX+2] = tracking_yaw;
        } else {
            x_init[i*NX]   = interpolated_x;
            x_init[i*NX+1] = interpolated_y;
            x_init[i*NX+2] = interpolated_yaw;
        }
    }

    printf("Initial guess for state 0: [%.3f, %.3f, %.3f]\n", x_init[0], x_init[1], x_init[2]);
    printf("Initial guess for state N: [%.3f, %.3f, %.3f]\n", x_init[N*NX], x_init[N*NX+1], x_init[N*NX+2]);

    for(int i=0; i<=N; i++) {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", &x_init[i*NX]);
    }

    double p[NP];
    for(int i=0; i<NP; i++) p[i] = (i < 520) ? costmap_data[i] : 0.0;
    for(int i=0; i<=N; i++) {
        Jackal_acados_update_params(acados_ocp_capsule, i, p, NP);
    }

    int NTIMINGS = std::max(10, num_steps);
    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);

    double xtraj[NX*(N+1)], utraj[NU*N];
    double exec_time = 0.0, kkt_norm_inf, elapsed_time;
    int sqp_iter;

    for(int i=0; i<NTIMINGS; i++) {
        status = Jackal_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        exec_time += elapsed_time;
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
        ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);
        if(status == ACADOS_SUCCESS) break;
    }

    for(int i=0; i<=N; i++) {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", &xtraj[i*NX]);
    }
    for(int i=0; i<N; i++) {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", &utraj[i*NU]);
    }

    printf("\n--- MPC Solution ---\n");
    printf("First 3 trajectory points:\n");
    for(int i=0; i<3; i++) {
        printf("Point %d: [%.6f, %.6f, %.6f]\n", i, xtraj[i*NX], xtraj[i*NX+1], xtraj[i*NX+2]);
    }
    printf("...\n");
    for(int i=0; i<=N; i+=5) {
        printf("Point %d: [%.3f, %.3f, %.3f]\n", i, xtraj[i*NX], xtraj[i*NX+1], xtraj[i*NX+2]);
    }

    results[0] = utraj[0];
    results[1] = -utraj[1];
    results[4] = sqp_iter;
    results[5] = exec_time*1000;
    results[6] = kkt_norm_inf;
    for(int i=0; i<33; i++) trajectory[i] = xtraj[i];

    return status;
}

int my_NMPC_solver::reset_solver() {
    int status = Jackal_acados_free(acados_ocp_capsule);
    if (status) printf("Jackal_acados_free() returned status %d.\n", status);
    status = Jackal_acados_free_capsule(acados_ocp_capsule);
    if (status) printf("Jackal_acados_free_capsule() returned status %d.\n", status);
    return status;
}