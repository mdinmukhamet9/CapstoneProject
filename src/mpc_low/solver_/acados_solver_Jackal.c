/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "Jackal_model/Jackal_model.h"
#include "Jackal_constraints/Jackal_constraints.h"



#include "acados_solver_Jackal.h"

#define NX     JACKAL_NX
#define NZ     JACKAL_NZ
#define NU     JACKAL_NU
#define NP     JACKAL_NP
#define NY0    JACKAL_NY0
#define NY     JACKAL_NY
#define NYN    JACKAL_NYN

#define NBX    JACKAL_NBX
#define NBX0   JACKAL_NBX0
#define NBU    JACKAL_NBU
#define NG     JACKAL_NG
#define NBXN   JACKAL_NBXN
#define NGN    JACKAL_NGN

#define NH     JACKAL_NH
#define NHN    JACKAL_NHN
#define NH0    JACKAL_NH0
#define NPHI   JACKAL_NPHI
#define NPHIN  JACKAL_NPHIN
#define NPHI0  JACKAL_NPHI0
#define NR     JACKAL_NR

#define NS     JACKAL_NS
#define NS0    JACKAL_NS0
#define NSN    JACKAL_NSN

#define NSBX   JACKAL_NSBX
#define NSBU   JACKAL_NSBU
#define NSH0   JACKAL_NSH0
#define NSH    JACKAL_NSH
#define NSHN   JACKAL_NSHN
#define NSG    JACKAL_NSG
#define NSPHI0 JACKAL_NSPHI0
#define NSPHI  JACKAL_NSPHI
#define NSPHIN JACKAL_NSPHIN
#define NSGN   JACKAL_NSGN
#define NSBXN  JACKAL_NSBXN



// ** solver data **

Jackal_solver_capsule * Jackal_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(Jackal_solver_capsule));
    Jackal_solver_capsule *capsule = (Jackal_solver_capsule *) capsule_mem;

    return capsule;
}


int Jackal_acados_free_capsule(Jackal_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int Jackal_acados_create(Jackal_solver_capsule* capsule)
{
    int N_shooting_intervals = JACKAL_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return Jackal_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int Jackal_acados_update_time_steps(Jackal_solver_capsule* capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "Jackal_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;
}

/**
 * Internal function for Jackal_acados_create: step 1
 */
void Jackal_acados_create_1_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP_RTI;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = FULL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = LINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = LINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = LINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = NO_REGULARIZE;
}


/**
 * Internal function for Jackal_acados_create: step 2
 */
ocp_nlp_dims* Jackal_acados_create_2_create_and_set_dimensions(Jackal_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 18
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;
    int* np  = intNp1mem + (N+1)*17;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
        np[i]     = NP;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    nbxe[0] = 3;
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "np", np);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);

    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for Jackal_acados_create: step 3
 */
void Jackal_acados_create_3_create_and_set_functions(Jackal_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;


    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_param_casadi_create(&capsule->__CAPSULE_FNC__ , 520); \
    } while(false)
    // constraints.constr_type == "BGH" and dims.nh > 0
    capsule->nl_constr_h_fun_jac = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++) {
        MAP_CASADI_FNC(nl_constr_h_fun_jac[i], Jackal_constr_h_fun_jac_uxt_zt);
    }
    capsule->nl_constr_h_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++) {
        MAP_CASADI_FNC(nl_constr_h_fun[i], Jackal_constr_h_fun);
    }
    



    // explicit ode
    capsule->forw_vde_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(forw_vde_casadi[i], Jackal_expl_vde_forw);
    }

    capsule->expl_ode_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_ode_fun[i], Jackal_expl_ode_fun);
    }



#undef MAP_CASADI_FNC
}


/**
 * Internal function for Jackal_acados_create: step 4
 */
void Jackal_acados_create_4_set_default_parameters(Jackal_solver_capsule* capsule) {
    const int N = capsule->nlp_solver_plan->N;
    // initialize parameters to nominal value
    double* p = calloc(NP, sizeof(double));

    for (int i = 0; i <= N; i++) {
        Jackal_acados_update_params(capsule, i, p, NP);
    }
    free(p);
}


/**
 * Internal function for Jackal_acados_create: step 5
 */
void Jackal_acados_create_5_set_nlp_in(Jackal_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    int tmp_int = 0;

    /************************************************
    *  nlp_in
    ************************************************/
//    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
//    capsule->nlp_in = nlp_in;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    // set up time_steps

    if (new_time_steps)
    {
        Jackal_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {double time_step = 0.025;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_step);
        }
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->forw_vde_casadi[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
    }

    /**** Cost ****/
    double* yref_0 = calloc(NY0, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);

   double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[0+(NY0) * 0] = 100;
    W_0[1+(NY0) * 1] = 100;
    W_0[3+(NY0) * 3] = 1000;
    W_0[4+(NY0) * 4] = 1000;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);
    double* Vx_0 = calloc(NY0*NX, sizeof(double));
    // change only the non-zero elements:
    Vx_0[0+(NY0) * 0] = 1;
    Vx_0[1+(NY0) * 1] = 1;
    Vx_0[2+(NY0) * 2] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vx", Vx_0);
    free(Vx_0);
    double* Vu_0 = calloc(NY0*NU, sizeof(double));
    // change only the non-zero elements:
    Vu_0[3+(NY0) * 0] = 1;
    Vu_0[4+(NY0) * 1] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vu", Vu_0);
    free(Vu_0);
    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(yref);
    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    W[0+(NY) * 0] = 100;
    W[1+(NY) * 1] = 100;
    W[3+(NY) * 3] = 1000;
    W[4+(NY) * 4] = 1000;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    free(W);
    double* Vx = calloc(NY*NX, sizeof(double));
    // change only the non-zero elements:
    Vx[0+(NY) * 0] = 1;
    Vx[1+(NY) * 1] = 1;
    Vx[2+(NY) * 2] = 1;
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vx", Vx);
    }
    free(Vx);

    
    double* Vu = calloc(NY*NU, sizeof(double));
    // change only the non-zero elements:
    
    Vu[3+(NY) * 0] = 1;
    Vu[4+(NY) * 1] = 1;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vu", Vu);
    }
    free(Vu);
    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);

    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    W_e[0+(NYN) * 0] = 100000;
    W_e[1+(NYN) * 1] = 100000;
    W_e[2+(NYN) * 2] = 100;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);
    double* Vx_e = calloc(NYN*NX, sizeof(double));
    // change only the non-zero elements:
    
    Vx_e[0+(NYN) * 0] = 1;
    Vx_e[1+(NYN) * 1] = 1;
    Vx_e[2+(NYN) * 2] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Vx", Vx_e);
    free(Vx_e);



    // slacks
    double* zlumem = calloc(4*NS, sizeof(double));
    double* Zl = zlumem+NS*0;
    double* Zu = zlumem+NS*1;
    double* zl = zlumem+NS*2;
    double* zu = zlumem+NS*3;
    // change only the non-zero elements:

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zl", Zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zu", Zu);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zl", zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zu", zu);
    }
    free(zlumem);



    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:
    lbx0[2] = 0.7853981633974483;
    ubx0[2] = 0.7853981633974483;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(3 * sizeof(int));
    
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);








    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    
    lbu[0] = -4.6;
    ubu[0] = 4.6;
    lbu[1] = -4.6;
    ubu[1] = 4.6;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);




    // set up soft bounds for nonlinear constraints
    int* idxsh = malloc(NSH * sizeof(int));
    
    idxsh[0] = 0;
    idxsh[1] = 1;
    idxsh[2] = 2;
    idxsh[3] = 3;
    idxsh[4] = 4;
    idxsh[5] = 5;
    idxsh[6] = 6;
    idxsh[7] = 7;
    idxsh[8] = 8;
    idxsh[9] = 9;
    idxsh[10] = 10;
    idxsh[11] = 11;
    idxsh[12] = 12;
    idxsh[13] = 13;
    idxsh[14] = 14;
    idxsh[15] = 15;
    idxsh[16] = 16;
    idxsh[17] = 17;
    idxsh[18] = 18;
    idxsh[19] = 19;
    idxsh[20] = 20;
    idxsh[21] = 21;
    idxsh[22] = 22;
    idxsh[23] = 23;
    idxsh[24] = 24;
    idxsh[25] = 25;
    idxsh[26] = 26;
    idxsh[27] = 27;
    idxsh[28] = 28;
    idxsh[29] = 29;
    idxsh[30] = 30;
    idxsh[31] = 31;
    idxsh[32] = 32;
    idxsh[33] = 33;
    idxsh[34] = 34;
    idxsh[35] = 35;
    idxsh[36] = 36;
    idxsh[37] = 37;
    idxsh[38] = 38;
    idxsh[39] = 39;
    idxsh[40] = 40;
    idxsh[41] = 41;
    idxsh[42] = 42;
    idxsh[43] = 43;
    idxsh[44] = 44;
    idxsh[45] = 45;
    idxsh[46] = 46;
    idxsh[47] = 47;
    idxsh[48] = 48;
    idxsh[49] = 49;
    idxsh[50] = 50;
    idxsh[51] = 51;
    idxsh[52] = 52;
    idxsh[53] = 53;
    idxsh[54] = 54;
    idxsh[55] = 55;
    idxsh[56] = 56;
    idxsh[57] = 57;
    idxsh[58] = 58;
    idxsh[59] = 59;
    idxsh[60] = 60;
    idxsh[61] = 61;
    idxsh[62] = 62;
    idxsh[63] = 63;
    idxsh[64] = 64;
    idxsh[65] = 65;
    idxsh[66] = 66;
    idxsh[67] = 67;
    idxsh[68] = 68;
    idxsh[69] = 69;
    idxsh[70] = 70;
    idxsh[71] = 71;
    idxsh[72] = 72;
    idxsh[73] = 73;
    idxsh[74] = 74;
    idxsh[75] = 75;
    idxsh[76] = 76;
    idxsh[77] = 77;
    idxsh[78] = 78;
    idxsh[79] = 79;
    idxsh[80] = 80;
    idxsh[81] = 81;
    idxsh[82] = 82;
    idxsh[83] = 83;
    idxsh[84] = 84;
    idxsh[85] = 85;
    idxsh[86] = 86;
    idxsh[87] = 87;
    idxsh[88] = 88;
    idxsh[89] = 89;
    idxsh[90] = 90;
    idxsh[91] = 91;
    idxsh[92] = 92;
    idxsh[93] = 93;
    idxsh[94] = 94;
    idxsh[95] = 95;
    idxsh[96] = 96;
    idxsh[97] = 97;
    idxsh[98] = 98;
    idxsh[99] = 99;
    idxsh[100] = 100;
    idxsh[101] = 101;
    idxsh[102] = 102;
    idxsh[103] = 103;
    idxsh[104] = 104;
    idxsh[105] = 105;
    idxsh[106] = 106;
    idxsh[107] = 107;
    idxsh[108] = 108;
    idxsh[109] = 109;
    idxsh[110] = 110;
    idxsh[111] = 111;
    idxsh[112] = 112;
    idxsh[113] = 113;
    idxsh[114] = 114;
    idxsh[115] = 115;
    idxsh[116] = 116;
    idxsh[117] = 117;
    idxsh[118] = 118;
    idxsh[119] = 119;
    idxsh[120] = 120;
    idxsh[121] = 121;
    idxsh[122] = 122;
    idxsh[123] = 123;
    idxsh[124] = 124;
    idxsh[125] = 125;
    idxsh[126] = 126;
    idxsh[127] = 127;
    idxsh[128] = 128;
    idxsh[129] = 129;
    idxsh[130] = 130;
    idxsh[131] = 131;
    idxsh[132] = 132;
    idxsh[133] = 133;
    idxsh[134] = 134;
    idxsh[135] = 135;
    idxsh[136] = 136;
    idxsh[137] = 137;
    idxsh[138] = 138;
    idxsh[139] = 139;
    idxsh[140] = 140;
    idxsh[141] = 141;
    idxsh[142] = 142;
    idxsh[143] = 143;
    idxsh[144] = 144;
    idxsh[145] = 145;
    idxsh[146] = 146;
    idxsh[147] = 147;
    idxsh[148] = 148;
    idxsh[149] = 149;
    idxsh[150] = 150;
    idxsh[151] = 151;
    idxsh[152] = 152;
    idxsh[153] = 153;
    idxsh[154] = 154;
    idxsh[155] = 155;
    idxsh[156] = 156;
    idxsh[157] = 157;
    idxsh[158] = 158;
    idxsh[159] = 159;
    idxsh[160] = 160;
    idxsh[161] = 161;
    idxsh[162] = 162;
    idxsh[163] = 163;
    idxsh[164] = 164;
    idxsh[165] = 165;
    idxsh[166] = 166;
    idxsh[167] = 167;
    idxsh[168] = 168;
    idxsh[169] = 169;
    idxsh[170] = 170;
    idxsh[171] = 171;
    idxsh[172] = 172;
    idxsh[173] = 173;
    idxsh[174] = 174;
    idxsh[175] = 175;
    idxsh[176] = 176;
    idxsh[177] = 177;
    idxsh[178] = 178;
    idxsh[179] = 179;
    idxsh[180] = 180;
    idxsh[181] = 181;
    idxsh[182] = 182;
    idxsh[183] = 183;
    idxsh[184] = 184;
    idxsh[185] = 185;
    idxsh[186] = 186;
    idxsh[187] = 187;
    idxsh[188] = 188;
    idxsh[189] = 189;
    idxsh[190] = 190;
    idxsh[191] = 191;
    idxsh[192] = 192;
    idxsh[193] = 193;
    idxsh[194] = 194;
    idxsh[195] = 195;
    idxsh[196] = 196;
    idxsh[197] = 197;
    idxsh[198] = 198;
    idxsh[199] = 199;
    idxsh[200] = 200;
    idxsh[201] = 201;
    idxsh[202] = 202;
    idxsh[203] = 203;
    idxsh[204] = 204;
    idxsh[205] = 205;
    idxsh[206] = 206;
    idxsh[207] = 207;
    idxsh[208] = 208;
    idxsh[209] = 209;
    idxsh[210] = 210;
    idxsh[211] = 211;
    idxsh[212] = 212;
    idxsh[213] = 213;
    idxsh[214] = 214;
    idxsh[215] = 215;
    idxsh[216] = 216;
    idxsh[217] = 217;
    idxsh[218] = 218;
    idxsh[219] = 219;
    idxsh[220] = 220;
    idxsh[221] = 221;
    idxsh[222] = 222;
    idxsh[223] = 223;
    idxsh[224] = 224;
    idxsh[225] = 225;
    idxsh[226] = 226;
    idxsh[227] = 227;
    idxsh[228] = 228;
    idxsh[229] = 229;
    idxsh[230] = 230;
    idxsh[231] = 231;
    idxsh[232] = 232;
    idxsh[233] = 233;
    idxsh[234] = 234;
    idxsh[235] = 235;
    idxsh[236] = 236;
    idxsh[237] = 237;
    idxsh[238] = 238;
    idxsh[239] = 239;
    idxsh[240] = 240;
    idxsh[241] = 241;
    idxsh[242] = 242;
    idxsh[243] = 243;
    idxsh[244] = 244;
    idxsh[245] = 245;
    idxsh[246] = 246;
    idxsh[247] = 247;
    idxsh[248] = 248;
    idxsh[249] = 249;
    idxsh[250] = 250;
    idxsh[251] = 251;
    idxsh[252] = 252;
    idxsh[253] = 253;
    idxsh[254] = 254;
    idxsh[255] = 255;
    idxsh[256] = 256;
    idxsh[257] = 257;
    idxsh[258] = 258;
    idxsh[259] = 259;
    double* lush = calloc(2*NSH, sizeof(double));
    double* lsh = lush;
    double* ush = lush + NSH;
    

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxsh", idxsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lsh", lsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ush", ush);
    }
    free(idxsh);
    free(lush);




    // x
    int* idxbx = malloc(NBX * sizeof(int));
    
    idxbx[0] = 0;
    idxbx[1] = 1;
    idxbx[2] = 2;
    double* lubx = calloc(2*NBX, sizeof(double));
    double* lbx = lubx;
    double* ubx = lubx + NBX;
    
    lbx[0] = -10;
    ubx[0] = 10;
    lbx[1] = -10;
    ubx[1] = 10;
    lbx[2] = -18.84955592153876;
    ubx[2] = 18.84955592153876;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
    }
    free(idxbx);
    free(lubx);




    // set up nonlinear constraints for stage 1 to N-1
    double* luh = calloc(2*NH, sizeof(double));
    double* lh = luh;
    double* uh = luh + NH;

    
    lh[0] = -1000000000;
    lh[1] = -1000000000;
    lh[2] = -1000000000;
    lh[3] = -1000000000;
    lh[4] = -1000000000;
    lh[5] = -1000000000;
    lh[6] = -1000000000;
    lh[7] = -1000000000;
    lh[8] = -1000000000;
    lh[9] = -1000000000;
    lh[10] = -1000000000;
    lh[11] = -1000000000;
    lh[12] = -1000000000;
    lh[13] = -1000000000;
    lh[14] = -1000000000;
    lh[15] = -1000000000;
    lh[16] = -1000000000;
    lh[17] = -1000000000;
    lh[18] = -1000000000;
    lh[19] = -1000000000;
    lh[20] = -1000000000;
    lh[21] = -1000000000;
    lh[22] = -1000000000;
    lh[23] = -1000000000;
    lh[24] = -1000000000;
    lh[25] = -1000000000;
    lh[26] = -1000000000;
    lh[27] = -1000000000;
    lh[28] = -1000000000;
    lh[29] = -1000000000;
    lh[30] = -1000000000;
    lh[31] = -1000000000;
    lh[32] = -1000000000;
    lh[33] = -1000000000;
    lh[34] = -1000000000;
    lh[35] = -1000000000;
    lh[36] = -1000000000;
    lh[37] = -1000000000;
    lh[38] = -1000000000;
    lh[39] = -1000000000;
    lh[40] = -1000000000;
    lh[41] = -1000000000;
    lh[42] = -1000000000;
    lh[43] = -1000000000;
    lh[44] = -1000000000;
    lh[45] = -1000000000;
    lh[46] = -1000000000;
    lh[47] = -1000000000;
    lh[48] = -1000000000;
    lh[49] = -1000000000;
    lh[50] = -1000000000;
    lh[51] = -1000000000;
    lh[52] = -1000000000;
    lh[53] = -1000000000;
    lh[54] = -1000000000;
    lh[55] = -1000000000;
    lh[56] = -1000000000;
    lh[57] = -1000000000;
    lh[58] = -1000000000;
    lh[59] = -1000000000;
    lh[60] = -1000000000;
    lh[61] = -1000000000;
    lh[62] = -1000000000;
    lh[63] = -1000000000;
    lh[64] = -1000000000;
    lh[65] = -1000000000;
    lh[66] = -1000000000;
    lh[67] = -1000000000;
    lh[68] = -1000000000;
    lh[69] = -1000000000;
    lh[70] = -1000000000;
    lh[71] = -1000000000;
    lh[72] = -1000000000;
    lh[73] = -1000000000;
    lh[74] = -1000000000;
    lh[75] = -1000000000;
    lh[76] = -1000000000;
    lh[77] = -1000000000;
    lh[78] = -1000000000;
    lh[79] = -1000000000;
    lh[80] = -1000000000;
    lh[81] = -1000000000;
    lh[82] = -1000000000;
    lh[83] = -1000000000;
    lh[84] = -1000000000;
    lh[85] = -1000000000;
    lh[86] = -1000000000;
    lh[87] = -1000000000;
    lh[88] = -1000000000;
    lh[89] = -1000000000;
    lh[90] = -1000000000;
    lh[91] = -1000000000;
    lh[92] = -1000000000;
    lh[93] = -1000000000;
    lh[94] = -1000000000;
    lh[95] = -1000000000;
    lh[96] = -1000000000;
    lh[97] = -1000000000;
    lh[98] = -1000000000;
    lh[99] = -1000000000;
    lh[100] = -1000000000;
    lh[101] = -1000000000;
    lh[102] = -1000000000;
    lh[103] = -1000000000;
    lh[104] = -1000000000;
    lh[105] = -1000000000;
    lh[106] = -1000000000;
    lh[107] = -1000000000;
    lh[108] = -1000000000;
    lh[109] = -1000000000;
    lh[110] = -1000000000;
    lh[111] = -1000000000;
    lh[112] = -1000000000;
    lh[113] = -1000000000;
    lh[114] = -1000000000;
    lh[115] = -1000000000;
    lh[116] = -1000000000;
    lh[117] = -1000000000;
    lh[118] = -1000000000;
    lh[119] = -1000000000;
    lh[120] = -1000000000;
    lh[121] = -1000000000;
    lh[122] = -1000000000;
    lh[123] = -1000000000;
    lh[124] = -1000000000;
    lh[125] = -1000000000;
    lh[126] = -1000000000;
    lh[127] = -1000000000;
    lh[128] = -1000000000;
    lh[129] = -1000000000;
    lh[130] = -1000000000;
    lh[131] = -1000000000;
    lh[132] = -1000000000;
    lh[133] = -1000000000;
    lh[134] = -1000000000;
    lh[135] = -1000000000;
    lh[136] = -1000000000;
    lh[137] = -1000000000;
    lh[138] = -1000000000;
    lh[139] = -1000000000;
    lh[140] = -1000000000;
    lh[141] = -1000000000;
    lh[142] = -1000000000;
    lh[143] = -1000000000;
    lh[144] = -1000000000;
    lh[145] = -1000000000;
    lh[146] = -1000000000;
    lh[147] = -1000000000;
    lh[148] = -1000000000;
    lh[149] = -1000000000;
    lh[150] = -1000000000;
    lh[151] = -1000000000;
    lh[152] = -1000000000;
    lh[153] = -1000000000;
    lh[154] = -1000000000;
    lh[155] = -1000000000;
    lh[156] = -1000000000;
    lh[157] = -1000000000;
    lh[158] = -1000000000;
    lh[159] = -1000000000;
    lh[160] = -1000000000;
    lh[161] = -1000000000;
    lh[162] = -1000000000;
    lh[163] = -1000000000;
    lh[164] = -1000000000;
    lh[165] = -1000000000;
    lh[166] = -1000000000;
    lh[167] = -1000000000;
    lh[168] = -1000000000;
    lh[169] = -1000000000;
    lh[170] = -1000000000;
    lh[171] = -1000000000;
    lh[172] = -1000000000;
    lh[173] = -1000000000;
    lh[174] = -1000000000;
    lh[175] = -1000000000;
    lh[176] = -1000000000;
    lh[177] = -1000000000;
    lh[178] = -1000000000;
    lh[179] = -1000000000;
    lh[180] = -1000000000;
    lh[181] = -1000000000;
    lh[182] = -1000000000;
    lh[183] = -1000000000;
    lh[184] = -1000000000;
    lh[185] = -1000000000;
    lh[186] = -1000000000;
    lh[187] = -1000000000;
    lh[188] = -1000000000;
    lh[189] = -1000000000;
    lh[190] = -1000000000;
    lh[191] = -1000000000;
    lh[192] = -1000000000;
    lh[193] = -1000000000;
    lh[194] = -1000000000;
    lh[195] = -1000000000;
    lh[196] = -1000000000;
    lh[197] = -1000000000;
    lh[198] = -1000000000;
    lh[199] = -1000000000;
    lh[200] = -1000000000;
    lh[201] = -1000000000;
    lh[202] = -1000000000;
    lh[203] = -1000000000;
    lh[204] = -1000000000;
    lh[205] = -1000000000;
    lh[206] = -1000000000;
    lh[207] = -1000000000;
    lh[208] = -1000000000;
    lh[209] = -1000000000;
    lh[210] = -1000000000;
    lh[211] = -1000000000;
    lh[212] = -1000000000;
    lh[213] = -1000000000;
    lh[214] = -1000000000;
    lh[215] = -1000000000;
    lh[216] = -1000000000;
    lh[217] = -1000000000;
    lh[218] = -1000000000;
    lh[219] = -1000000000;
    lh[220] = -1000000000;
    lh[221] = -1000000000;
    lh[222] = -1000000000;
    lh[223] = -1000000000;
    lh[224] = -1000000000;
    lh[225] = -1000000000;
    lh[226] = -1000000000;
    lh[227] = -1000000000;
    lh[228] = -1000000000;
    lh[229] = -1000000000;
    lh[230] = -1000000000;
    lh[231] = -1000000000;
    lh[232] = -1000000000;
    lh[233] = -1000000000;
    lh[234] = -1000000000;
    lh[235] = -1000000000;
    lh[236] = -1000000000;
    lh[237] = -1000000000;
    lh[238] = -1000000000;
    lh[239] = -1000000000;
    lh[240] = -1000000000;
    lh[241] = -1000000000;
    lh[242] = -1000000000;
    lh[243] = -1000000000;
    lh[244] = -1000000000;
    lh[245] = -1000000000;
    lh[246] = -1000000000;
    lh[247] = -1000000000;
    lh[248] = -1000000000;
    lh[249] = -1000000000;
    lh[250] = -1000000000;
    lh[251] = -1000000000;
    lh[252] = -1000000000;
    lh[253] = -1000000000;
    lh[254] = -1000000000;
    lh[255] = -1000000000;
    lh[256] = -1000000000;
    lh[257] = -1000000000;
    lh[258] = -1000000000;
    lh[259] = -1000000000;

    

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                      &capsule->nl_constr_h_fun_jac[i-1]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                      &capsule->nl_constr_h_fun[i-1]);
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "uh", uh);
    }
    free(luh);



    /* terminal constraints */

    // set up bounds for last stage
    // x
    int* idxbx_e = malloc(NBXN * sizeof(int));
    
    idxbx_e[0] = 0;
    idxbx_e[1] = 1;
    idxbx_e[2] = 2;
    double* lubx_e = calloc(2*NBXN, sizeof(double));
    double* lbx_e = lubx_e;
    double* ubx_e = lubx_e + NBXN;
    
    lbx_e[0] = -10;
    ubx_e[0] = 10;
    lbx_e[1] = -10;
    ubx_e[1] = 10;
    lbx_e[2] = -18.84955592153876;
    ubx_e[2] = 18.84955592153876;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "idxbx", idxbx_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lbx", lbx_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "ubx", ubx_e);
    free(idxbx_e);
    free(lubx_e);












}


/**
 * Internal function for Jackal_acados_create: step 6
 */
void Jackal_acados_create_6_set_opts(Jackal_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/

int fixed_hess = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "fixed_hess", &fixed_hess);
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization", "fixed_step");int with_solution_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params", &with_solution_sens_wrt_params);

    int with_value_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_value_sens_wrt_params", &with_value_sens_wrt_params);

    int full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "full_step_dual", &full_step_dual);

    // set collocation type (relevant for implicit integrators)
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_collocation_type", &collocation_type);

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 1;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps);

    // set up sim_method_num_stages
    // all sim_method_num_stages are identical
    int sim_method_num_stages = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);

    // set up sim_method_jac_reuse
    bool tmp_bool = (bool) 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0.00001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */

    int nlp_solver_ext_qp_res = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");


    int as_rti_iter = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "as_rti_iter", &as_rti_iter);

    int as_rti_level = 4;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "as_rti_level", &as_rti_level);

    int rti_log_residuals = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_log_residuals", &rti_log_residuals);

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);


    int qp_solver_warm_start = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_warm_start", &qp_solver_warm_start);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);

    int ext_cost_num_hess = 0;



}


/**
 * Internal function for Jackal_acados_create: step 7
 */
void Jackal_acados_create_7_set_nlp_out(Jackal_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    
    x0[2] = 0.7853981633974483;


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for Jackal_acados_create: step 8
 */
//void Jackal_acados_create_8_create_solver(Jackal_solver_capsule* capsule)
//{
//    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
//}

/**
 * Internal function for Jackal_acados_create: step 9
 */
int Jackal_acados_create_9_precompute(Jackal_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int Jackal_acados_create_with_discretization(Jackal_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != JACKAL_N && !new_time_steps) {
        fprintf(stderr, "Jackal_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, JACKAL_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    Jackal_acados_create_1_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 3) create and set dimensions
    capsule->nlp_dims = Jackal_acados_create_2_create_and_set_dimensions(capsule);
    Jackal_acados_create_3_create_and_set_functions(capsule);

    // 4) set default parameters in functions
    Jackal_acados_create_4_set_default_parameters(capsule);

    // 5) create and set nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);
    Jackal_acados_create_5_set_nlp_in(capsule, N, new_time_steps);

    // 6) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    Jackal_acados_create_6_set_opts(capsule);

    // 7) create and set nlp_out
    // 7.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 7.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    Jackal_acados_create_7_set_nlp_out(capsule);

    // 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
    //Jackal_acados_create_8_create_solver(capsule);

    // 9) do precomputations
    int status = Jackal_acados_create_9_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int Jackal_acados_update_qp_solver_cond_N(Jackal_solver_capsule* capsule, int qp_solver_cond_N)
{
    printf("\nacados_update_qp_solver_cond_N() not implemented, since no partial condensing solver is used!\n\n");
    exit(1);
    return -1;
}


int Jackal_acados_reset(Jackal_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+2*NS0+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NH0+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "t", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "pi", buffer);
        }
    }

    free(buffer);
    return 0;
}




int Jackal_acados_update_params(Jackal_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 520;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }

    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->forw_vde_casadi[stage].set_param(capsule->forw_vde_casadi+stage, p);
        capsule->expl_ode_fun[stage].set_param(capsule->expl_ode_fun+stage, p);

        // constraints
        if (stage == 0)
        {
        }
        else
        {
            capsule->nl_constr_h_fun_jac[stage-1].set_param(capsule->nl_constr_h_fun_jac+stage-1, p);
            capsule->nl_constr_h_fun[stage-1].set_param(capsule->nl_constr_h_fun+stage-1, p);
        }

        // cost
        if (stage == 0)
        {
        }
        else // 0 < stage < N
        {
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        // constraints
    }

    return solver_status;
}


int Jackal_acados_update_params_sparse(Jackal_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    int solver_status = 0;

    int casadi_np = 520;
    if (casadi_np < n_update) {
        printf("Jackal_acados_update_params_sparse: trying to set %d parameters for external functions."
            " External function has %d parameters. Exiting.\n", n_update, casadi_np);
        exit(1);
    }
    // for (int i = 0; i < n_update; i++)
    // {
    //     if (idx[i] > casadi_np) {
    //         printf("Jackal_acados_update_params_sparse: attempt to set parameters with index %d, while"
    //             " external functions only has %d parameters. Exiting.\n", idx[i], casadi_np);
    //         exit(1);
    //     }
    //     printf("param %d value %e\n", idx[i], p[i]);
    // }
    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->forw_vde_casadi[stage].set_param_sparse(capsule->forw_vde_casadi+stage, n_update, idx, p);
        capsule->expl_ode_fun[stage].set_param_sparse(capsule->expl_ode_fun+stage, n_update, idx, p);

        // constraints
        if (stage == 0)
        {
        }
        else
        {
            capsule->nl_constr_h_fun_jac[stage-1].set_param_sparse(capsule->nl_constr_h_fun_jac+stage-1, n_update, idx, p);
            capsule->nl_constr_h_fun[stage-1].set_param_sparse(capsule->nl_constr_h_fun+stage-1, n_update, idx, p);
        }

        // cost
        if (stage == 0)
        {
        }
        else // 0 < stage < N
        {
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        // constraints
    }


    return solver_status;
}

int Jackal_acados_solve(Jackal_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}


void Jackal_acados_batch_solve(Jackal_solver_capsule ** capsules, int N_batch)
{

    for (int i = 0; i < N_batch; i++)
    {
        ocp_nlp_solve(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out);
    }


    return;
}


int Jackal_acados_free(Jackal_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_param_casadi_free(&capsule->forw_vde_casadi[i]);
        external_function_param_casadi_free(&capsule->expl_ode_fun[i]);
    }
    free(capsule->forw_vde_casadi);
    free(capsule->expl_ode_fun);

    // cost

    // constraints
    for (int i = 0; i < N-1; i++)
    {
        external_function_param_casadi_free(&capsule->nl_constr_h_fun_jac[i]);
        external_function_param_casadi_free(&capsule->nl_constr_h_fun[i]);
    }
    free(capsule->nl_constr_h_fun_jac);
    free(capsule->nl_constr_h_fun);

    return 0;
}


void Jackal_acados_print_stats(Jackal_solver_capsule* capsule)
{
    int nlp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "nlp_iter", &nlp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    
    double stat[1200];
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "statistics", stat);

    int nrow = nlp_iter+1 < stat_m ? nlp_iter+1 : stat_m;


    printf("iter\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            tmp_int = (int) stat[i + j * nrow];
            printf("%d\t", tmp_int);
        }
        printf("\n");
    }
}

int Jackal_acados_custom_update(Jackal_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *Jackal_acados_get_nlp_in(Jackal_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *Jackal_acados_get_nlp_out(Jackal_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *Jackal_acados_get_sens_out(Jackal_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *Jackal_acados_get_nlp_solver(Jackal_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *Jackal_acados_get_nlp_config(Jackal_solver_capsule* capsule) { return capsule->nlp_config; }
void *Jackal_acados_get_nlp_opts(Jackal_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *Jackal_acados_get_nlp_dims(Jackal_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *Jackal_acados_get_nlp_plan(Jackal_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
