#ifndef ACADOS_SOLVER_HOLDER_HPP_
#define ACADOS_SOLVER_HOLDER_HPP_

// Standard
#include <stdio.h>
#include <stdlib.h>
// Acados
#include <acados/utils/print.h>
#include <acados/utils/math.h>
#include <acados_c/ocp_nlp_interface.h>
#include <acados_c/external_function_interface.h>
#include <acados_solver_Jackal.h>
// Blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include <math.h>
#include <eigen3/Eigen/Dense>

Eigen::MatrixXf get_cpose(float q1, float q2, float q3, float q4, float q5, float q6, float q7);

Eigen::MatrixXf get_velocity(float q1, float q2, float q3, float q4, float q5, float q6, float q7,
                             float u_1, float u_2, float u_3, float u_4, float u_5, float u_6, float u_7);

#define NX     JACKAL_NX
#define NZ     JACKAL_NZ
#define NU     JACKAL_NU
#define NP     JACKAL_NP
#define NBX    JACKAL_NBX
#define NBX0   JACKAL_NBX0
#define NBU    JACKAL_NBU
#define NSBX   JACKAL_NSBX
#define NSBU   JACKAL_NSBU
#define NSH    JACKAL_NSH
#define NSG    JACKAL_NSG
#define NSPHI  JACKAL_NSPHI
#define NSHN   JACKAL_NSHN
#define NSGN   JACKAL_NSGN
#define NSPHIN JACKAL_NSPHIN
#define NSBXN  JACKAL_NSBXN
#define NS     JACKAL_NS
#define NSN    JACKAL_NSN
#define NG     JACKAL_NG
#define NBXN   JACKAL_NBXN
#define NGN    JACKAL_NGN
#define NY0    JACKAL_NY0
#define NY     JACKAL_NY
#define NYN    JACKAL_NYN
#define NH     JACKAL_NH
#define NPHI   JACKAL_NPHI
#define NHN    JACKAL_NHN
#define NPHIN  JACKAL_NPHIN
#define NR     JACKAL_NR

class my_NMPC_solver {
private:
    int num_steps;
    Jackal_solver_capsule *acados_ocp_capsule;

public:
    my_NMPC_solver(int n);
    int solve_my_mpc(double current_robot_position[3], double costmap_data[520], double current_robot_goal[3], double tracking_goal[30], double results[8], double trajectory[33]);
    int reset_solver();
};

#endif // ACADOS_SOLVER_HOLDER_HPP_