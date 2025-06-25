#include "acados_solver_holder.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}


my_NMPC_solver::my_NMPC_solver(int n) {
    num_steps = n;
    Jackal_solver_capsule *my_acados_ocp_capsule = Jackal_acados_create_capsule();
    acados_ocp_capsule = my_acados_ocp_capsule;
    int N = JACKAL_N; //JACKAL_N;
    double* new_time_steps = NULL;
    int status = Jackal_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);
    if (status) {
        printf("Jackal_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }
}

int my_NMPC_solver::solve_my_mpc(double current_robot_position[3], double costmap_data[520], 
                                double current_robot_goal[3], double tracking_goal[30],
                                double results[8], double trajectory[33], double weights[5],
                                bool person_detected, double person_pos[3]) {
    // printf("\n--- MPC Inputs ---\n");
    // printf("Start: [%.3f, %.3f, %.3f]\n", current_robot_position[0], current_robot_position[1], current_robot_position[2]);
    // printf("Goal (relative): [%.3f, %.3f, %.3f]\n", current_robot_goal[0], current_robot_goal[1], current_robot_goal[2]);

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
        lbx0[i] = current_robot_position[i];
        ubx0[i] = current_robot_position[i];
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    double pi = M_PI;
    double lbx[3] = {-10.0, -10.0, -2*M_PI};
    double ubx[3] = {+10.0, +10.0, 2*M_PI};
    double lbu[2] = {-10, -10};
    double ubu[2] = {10, 10};

    double y_ref[NY] = {0.0};
    y_ref[0] = current_robot_goal[0];
    y_ref[1] = current_robot_goal[1];
    y_ref[2] = current_robot_goal[2]; //normalize_angle(absolute_goal[2]);
    y_ref[3] = 1.0;
    y_ref[4] = 0.0;

    double y_ref_N[NYN] = {0.0};
    for(int i=0; i<NYN; i++) y_ref_N[i] = current_robot_goal[i];

    double W[(NU+NX)*(NU+NX)] = {0.0};
    double WN[NX*NX] = {0.0};

    W[0] = weights[0];
    W[1*(NU+NX)+1] = weights[1];
    W[2*(NU+NX)+2] = weights[2];
    W[3*(NU+NX)+3] = 5.0;
    W[4*(NU+NX)+4] = 5.0;

    WN[0] = weights[3];       // terminal weight for x
    WN[NX+1] = weights[4];    // terminal weight for y (new weight)
    WN[2*NX+2] = weights[5];  // terminal weight for theta (new weight)

    double lsh[NSH] = {0.0}, ush[NSH] = {0.0};
    const double r_base = 0.254;   
    const double r_obst = 0.07;     
    const double min_distance = r_base + r_obst; // 0.324 m
    const double a = 0.5; 


    double p[NP];
    for (int i = 0; i < NP; i++) {
        p[i] = costmap_data[i];
    }

    for(int i = 0; i <= N; i++) {
        Jackal_acados_update_params(acados_ocp_capsule, i, p, NP);
    }

    for(int stage = 0; stage < N; stage++) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "yref", y_ref);

        double x_pred[3] = {0.0};
        double x_init_stage[NX] = {0.0};
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage, "x", x_init_stage);
        x_pred[0] = x_init_stage[0];
        x_pred[1] = x_init_stage[1];
        x_pred[2] = x_init_stage[2];

        double robot_x = current_robot_position[0] + x_pred[0];
        double robot_y = current_robot_position[1] + x_pred[1];

        // Obstacle avoidance constraints for 260 obstacles
        for (int k = 0; k < 260; k++) {
            double obs_x = p[k * 2];
            double obs_y = p[k * 2 + 1];
            if (obs_x == 0.0 && obs_y == 0.0) { // Skip empty entries
                lsh[k] = 0.0;
                ush[k] = 0.0;
                continue;
            }
            double dx = robot_x - obs_x;
            double dy = robot_y - obs_y;
            double distance_squared = dx * dx + dy * dy;
            double min_distance_squared = min_distance * min_distance;

            if (distance_squared < min_distance_squared) {
                lsh[k] = -1e8; // Heavy penalty for violation
                ush[k] = 0.0;  // Upper bound as per notebook
                printf("Stage %d, Obstacle %d: Safety violation, distance=%.3f, min_distance=%.3f\n", stage, k, sqrt(distance_squared), min_distance);
            } else {
                lsh[k] = 0.0;
                ush[k] = 0.0;
            }
        }

        // // Dynamic speed adjustment for human (if detected)
        // if (person_detected) {
        //     double dx = robot_x - person_pos[0];
        //     double dy = robot_y - person_pos[1];
        //     double dist_to_human = sqrt(dx * dx + dy * dy);

        //     double u_stage[NU] = {0.0};
        //     ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, stage, "u", u_stage);
        //     double phi_dot_1 = u_stage[0];
        //     double phi_dot_2 = u_stage[1];
        //     double linear_vel = (0.1 / 2.0) * (phi_dot_1 + phi_dot_2);
        //     double v_squared = linear_vel * linear_vel;

        //     double max_v_squared = dist_to_human * dist_to_human * a * a;
        //     if (v_squared > max_v_squared) {
        //         double speed_violation = v_squared - max_v_squared;
        //         W[18] += speed_violation; 
        //         W[24] += speed_violation; 
        //         printf("Stage %d: Speed violation, v^2=%.3f, max_v^2=%.3f, penalty=%.3f\n", stage, v_squared, max_v_squared, speed_violation * 100.0);
        //     }
        // }

        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "lsh", lsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "ush", ush);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, stage, "idxsh", idxsh);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, stage, "W", W);
    }

    y_ref_N[0] = current_robot_goal[0];
    y_ref_N[1] = current_robot_goal[1];
    y_ref_N[2] = current_robot_goal[2];
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", WN);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", y_ref_N);

    double x_init[NX*(N+1)]={0.0};

    // printf("Initial guess for state 0: [%.3f, %.3f, %.3f]\n", x_init[0], x_init[1], x_init[2]);
    // printf("Initial guess for state N: [%.3f, %.3f, %.3f]\n", x_init[N*NX], x_init[N*NX+1], x_init[N*NX+2]);

    for(int i=0; i<=N; i++) {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", &x_init[i*NX]);
    }

    // double p[NP];
    // for(int i=0; i<NP; i++) p[i] = 100.0;
    // for(int i=0; i<=N; i++) {
    //     Jackal_acados_update_params(acados_ocp_capsule, i, p, NP);
    // }

    int NTIMINGS = 1;//std::max(10, num_steps);
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
        if(status == ACADOS_SUCCESS) {
            ocp_nlp_eval_cost(nlp_solver, nlp_in, nlp_out);
            double ocp_cost;
            ocp_nlp_get(nlp_config, nlp_solver, "cost_value", &ocp_cost);
            // if (ocp_cost < 0.001) {
            //     printf("Early termination - low cost %.4f\n", ocp_cost);
            //     break;
            // }
            break;
        }
    }

    for(int i=0; i<=N; i++) {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", &xtraj[i*NX]);
    }
    for(int i=0; i<N; i++) {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", &utraj[i*NU]);
    }

    // printf("EXECUTION TIME: %3f ms \n", exec_time*1000 );
    // printf("\n--- MPC Solution ---\n");
    // printf("Control commands: linear=%.3f, angular=%.3f\n", utraj[0], utraj[1]);
    // printf("First 10 trajectory points:\n");
    // for(int i=0; i<11; i++) {
    //     printf("Point %d: [%.6f, %.6f, %.6f]\n", i, xtraj[i*NX], xtraj[i*NX+1], xtraj[i*NX+2]);
    // }
    // printf("\n--- Optimization Result ---\n");
    // printf("Final state trajectory:\n");
    // for (int ii = 0; ii <= nlp_dims->N; ii++) {
    //     ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    //     printf("Stage %2d x: [", ii);
    //     for (int j = 0; j < NX; j++) {
    //         printf("%6.3f", xtraj[ii*NX + j]);
    //         if (j < NX-1) printf(", ");
    //     }
    //     printf("]\n");
    // }

    // printf("\nOptimal controls:\n");
    // for (int ii = 0; ii < nlp_dims->N; ii++) {
    //     ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);
    //     printf("Stage %2d u: [", ii);
    //     for (int j = 0; j < NU; j++) {
    //         printf("%6.3f", utraj[ii*NU + j]);
    //         if (j < NU-1) printf(", ");
    //     }
    //     printf("]\n");
    // }
    double linear_vel = (0.1 / (2.0)) * (utraj[0] + utraj[1]); // r_wheel=0.1, alpha = 1.5
    double angular_vel = (0.1 / (1.5 * 0.3765)) * (utraj[0] - utraj[1]); // B=0.1615
    // printf("\n--- MPC Solution ---\n");
    printf("Control commands: phi_dot_1=%.3f, phi_dot_2=%.3f, linear=%.3f, angular=%.3f\n", 
        utraj[0], utraj[1], linear_vel, angular_vel);
    results[0] = utraj[0];
    results[1] = utraj[1];
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
